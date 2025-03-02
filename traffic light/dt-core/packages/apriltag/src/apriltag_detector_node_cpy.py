#!/usr/bin/env python3

import cv2
import rospy
import tf
import numpy as np

from threading import Thread
from concurrent.futures import ThreadPoolExecutor
from turbojpeg import TurboJPEG, TJPF_GRAY
from image_geometry import PinholeCameraModel
from dt_apriltags import Detector

from dt_class_utils import DTReminder
from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType

from duckietown_msgs.msg import AprilTagDetectionArray, AprilTagDetection, Segment, SegmentList
from sensor_msgs.msg import CameraInfo, CompressedImage
from geometry_msgs.msg import Transform, Vector3, Quaternion
import os


class AprilTagDetector(DTROS):

    def __init__(self):
        super(AprilTagDetector, self).__init__(
            node_name='apriltag_detector_node',
            node_type=NodeType.PERCEPTION
        )
        # get static parameters
        self.family = rospy.get_param('~family', 'tag36h11')
        self.ndetectors = rospy.get_param('~ndetectors', 1)
        self.nthreads = rospy.get_param('~nthreads', 1)
        self.quad_decimate = rospy.get_param('~quad_decimate', 1.0)
        self.quad_sigma = rospy.get_param('~quad_sigma', 0.0)
        self.refine_edges = rospy.get_param('~refine_edges', 1)
        self.decode_sharpening = rospy.get_param('~decode_sharpening', 0.25)
        self.tag_size = rospy.get_param('~tag_size', 0.065)
        self.rectify_alpha = rospy.get_param('~rectify_alpha', 0.0)
        ###########################
        self.start_s = round(rospy.Time.now().to_sec(), 1)
        # self.bots_in_last_n_frames = np.array([[], [], [], []])
        self.bots_in_last_frame = np.array([])
        self.bots_first_obs_times = {}  # the time a bot that is first observed by the camera
        self.bots_obs_times = {}  # total time a bot has been observed by the camera
        self.total_col = rospy.get_param('/camera_node/res_w', 1296)
        self.total_row = rospy.get_param('/camera_node/res_h', 1296)
        self.hor_lines = [[(0, int(2*self.total_row/6)), (int(self.total_col), int(2*self.total_row/6))],
                          [(0, int(3*self.total_row/6)), (int(self.total_col), int(3*self.total_row/6))],
                          [(0, int(4*self.total_row/6)), (int(self.total_col), int(4*self.total_row/6))]]
        self.ver_lines = [[(int(2*self.total_col/6), 0), (int(2*self.total_col/6), int(self.total_row))],
                          [(int(3*self.total_col/6), 0), (int(3*self.total_col/6), int(self.total_row))],
                          [(int(4*self.total_col/6), 0), (int(4*self.total_col/6), int(self.total_row))]]
        self.interpoints = [[int(2*self.total_col/6), int(2*self.total_row/6)], [int(4*self.total_col/6), int(4*self.total_row/6)]]
        self.lane_bots = {2:[], 4:[], 6:[], 8:[]}
        self.stoplines_centers = {2:[int(4/12*self.total_col), int(7/12*self.total_row)],
                                  4:[int(7/12*self.total_col), int(8/12*self.total_row)],
                                  6:[int(8/12*self.total_col), int(5/12*self.total_row)],
                                  8:[int(5/12*self.total_col), int(4/12*self.total_row)]}
        self.intersection_bots_n_frames = [[], [], [], [], [], [], [], [], [], [], [], [], [], [], [], [], []]
        self.througput = 0
        self.total_waiting_time = 0.
        self.veh = os.environ['VEHICLE_NAME']
        self.output_dir = "/code/catkin_ws/src/dt-core/camera_frames/"+str(self.start_s)# place to save camera data
        self.limited_sec = 60 # record limited_sec video
        os.makedirs(self.output_dir, exist_ok=True)
        ############################
        # dynamic parameter
        self.detection_freq = DTParam(
            '~detection_freq',
            default=-1,
            param_type=ParamType.INT,
            min_value=-1,
            max_value=30
        )
        self._detection_reminder = DTReminder(frequency=self.detection_freq.value)
        # camera info
        self._camera_parameters = None
        self._mapx, self._mapy = None, None
        # create detector object
        self._detectors = [Detector(
            families=self.family,
            nthreads=self.nthreads,
            quad_decimate=self.quad_decimate,
            quad_sigma=self.quad_sigma,
            refine_edges=self.refine_edges,
            decode_sharpening=self.decode_sharpening
        ) for _ in range(self.ndetectors)]
        self._renderer_busy = False
        # create a CV bridge object
        self._jpeg = TurboJPEG()
        # create subscribers
        self._img_sub = rospy.Subscriber(
            '~image',
            CompressedImage,
            self._img_cb,
            queue_size=1,
            buff_size='20MB'
        )
        self._cinfo_sub = rospy.Subscriber(
            '~camera_info',
            CameraInfo,
            self._cinfo_cb,
            queue_size=1
        )
        # create publisher
        self._tag_pub = rospy.Publisher(
            '~detections',
            AprilTagDetectionArray,
            queue_size=1,
            dt_topic_type=TopicType.PERCEPTION,
            dt_help='Tag detections',
        )
        self._img_pub = rospy.Publisher(
            '~detections/image/compressed',
            CompressedImage,
            queue_size=1,
            dt_topic_type=TopicType.VISUALIZATION,
            dt_help='Camera image with tag publishs superimposed',
        )

        ###################################################################
        self.pub_lane_bots = rospy.Publisher(
            "~detections/lane_bots", SegmentList, queue_size=1, dt_topic_type=TopicType.PERCEPTION
        )
        self.pub_bots_obs_times = rospy.Publisher(
            "~detections/bots_obs_times", SegmentList, queue_size=1, dt_topic_type=TopicType.PERCEPTION
        )

        self.sub_traffic_light_pattern = rospy.Subscriber(f'/{self.veh}/traffic_light_node/traffic_light_LED', Segment, self.cbTrafficLightPattern)
        self.TrafficLightPattern = "26"
        ###################################################################

        # create thread pool
        self._workers = ThreadPoolExecutor(self.ndetectors)
        self._tasks = [None] * self.ndetectors
        # create TF broadcaster
        self._tf_bcaster = tf.TransformBroadcaster()

    def on_shutdown(self):
        self.loginfo('Shutting down workers pool')
        self._workers.shutdown()

    def _cinfo_cb(self, msg):
        ##############################################
        # test the frequency of this call back function
        rospy.loginfo('Camera info received')
        ##############################################
        # create mapx and mapy
        H, W = msg.height, msg.width
        # create new camera info
        self.camera_model = PinholeCameraModel()
        self.camera_model.fromCameraInfo(msg)
        # find optimal rectified pinhole camera
        with self.profiler('/cb/camera_info/get_optimal_new_camera_matrix'):
            rect_K, _ = cv2.getOptimalNewCameraMatrix(
                self.camera_model.K,
                self.camera_model.D,
                (W, H),
                self.rectify_alpha
            )
            # store new camera parameters
            self._camera_parameters = (rect_K[0, 0], rect_K[1, 1], rect_K[0, 2], rect_K[1, 2])
        # create rectification map
        with self.profiler('/cb/camera_info/init_undistort_rectify_map'):
            self._mapx, self._mapy = cv2.initUndistortRectifyMap(
                self.camera_model.K,
                self.camera_model.D,
                None,
                rect_K,
                (W, H),
                cv2.CV_32FC1
            )
        # once we got the camera info, we can stop the subscriber
        self.loginfo('Camera info message received. Unsubscribing from camera_info topic.')
        # noinspection PyBroadException
        try:
            self._cinfo_sub.shutdown()
        except BaseException:
            pass

    def _detect(self, detector_id, msg):
        # turn image message into grayscale image
        with self.profiler('/cb/image/decode'):
            img = self._jpeg.decode(msg.data, pixel_format=TJPF_GRAY)
        # run input image through the rectification map
        with self.profiler('/cb/image/rectify'):
            img = cv2.remap(img, self._mapx, self._mapy, cv2.INTER_NEAREST)
        # detect tags
        with self.profiler('/cb/image/detection'):
            tags = self._detectors[detector_id].detect(
                img, True, self._camera_parameters, self.tag_size)
        # pack detections into a message
        tags_msg = AprilTagDetectionArray()
        tags_msg.header.stamp = msg.header.stamp
        tags_msg.header.frame_id = msg.header.frame_id
        ############################
        num_bots = 0
        current_bots_ids = []
        current_inter_bots_ids = []
        current_bots_centers = {}
        ############################
        for tag in tags:
            num_bots += 1
            # turn rotation matrix into quaternion
            q = _matrix_to_quaternion(tag.pose_R)
            p = tag.pose_t.T[0]

            ####################################################
            current_timee = rospy.Time.now().to_sec() - self.start_s
            current_bots_ids.append(tag.tag_id)
            # record the first time this bot is observed
            is_first_obs = True
            # check if the current bot was observed in last n frames.
            # # If the current bot is observed in the last n frames, then, this is not its first observation
            # for bots in self.bots_in_last_n_frames:
            #     if tag.tag_id in bots:
            #         is_first_obs = False
            #         break
            center_point = [int((tag.corners[0, 0].astype(int) + tag.corners[1, 0].astype(int)) * 0.5),
                            int((tag.corners[0, 1].astype(int) + tag.corners[1, 1].astype(int)) * 0.5)]

            current_bots_centers[tag.tag_id] = center_point

            if tag.tag_id in self.bots_in_last_frame: is_first_obs = False # if the bot is in the last frame, then it is not first observed
            if is_first_obs:  # if the bot is not in the last frame, then it is first observed
                self.bots_first_obs_times[tag.tag_id] = current_timee
                self.bots_obs_times[tag.tag_id] = 0
                if center_point[0] <= int(self.total_col/3) and center_point[1] >= int(self.total_row/2): # the bot is in lane 2
                    if tag.tag_id not in self.lane_bots[2]: self.lane_bots[2].append(tag.tag_id) # if the bot is not in lane 2 previously, then add it
                elif center_point[0] >= int(self.total_col/2) and center_point[1] >= int(2*self.total_row/3): # the bot is in lane 4
                    if tag.tag_id not in self.lane_bots[4]: self.lane_bots[4].append(tag.tag_id)
                elif center_point[0] >= int(2*self.total_col/3) and center_point[1] <= int(self.total_row/2):  # the bot is in lane 6
                    if tag.tag_id not in self.lane_bots[6]: self.lane_bots[6].append(tag.tag_id)
                elif center_point[0] <= int(self.total_col/2) and center_point[1] <= int(self.total_row/3):  # the bot is in lane 8
                    if tag.tag_id not in self.lane_bots[8]: self.lane_bots[8].append(tag.tag_id)
            else:  # the bot was observed in the last frame
                self.bots_obs_times[tag.tag_id] = round(current_timee - self.bots_first_obs_times[tag.tag_id], 1)

            # if the bot is inside the region of intersection
            if center_point[0] > self.interpoints[0][0] and center_point[0] < self.interpoints[1][0] and center_point[1] > self.interpoints[0][1] and center_point[1] < self.interpoints[1][1]:
                current_inter_bots_ids.append(tag.tag_id)
            ####################################################

            # create single tag detection object
            detection = AprilTagDetection(
                transform=Transform(
                    translation=Vector3(
                        x=p[0],
                        y=p[1],
                        z=p[2]
                    ),
                    rotation=Quaternion(
                        x=q[0],
                        y=q[1],
                        z=q[2],
                        w=q[3]
                    )
                ),
                tag_id=tag.tag_id,
                tag_family=str(tag.tag_family),
                hamming=tag.hamming,
                decision_margin=tag.decision_margin,
                homography=tag.homography.flatten().astype(np.float32).tolist(),
                center=tag.center.tolist(),
                corners=tag.corners.flatten().tolist(),
                pose_error=tag.pose_err
            )
            # add detection to array
            tags_msg.detections.append(detection)
            # publish tf
            self._tf_bcaster.sendTransform(
                p.tolist(),
                q.tolist(),
                msg.header.stamp,
                'tag/{:s}'.format(str(tag.tag_id)),
                msg.header.frame_id
            )

        ############################################################
        current_bots_ids = np.array(current_bots_ids)
        current_inter_bots_ids = np.array(current_inter_bots_ids)
        disappear_bots = np.setdiff1d(self.bots_in_last_frame, current_bots_ids)
        # if a bot disappears in the camera view, then we clean its memory
        for dis_tag_id in disappear_bots:
            try:
                self.bots_first_obs_times.pop(dis_tag_id)
                self.total_waiting_time += self.bots_obs_times[int(dis_tag_id)]
                self.bots_obs_times.pop(dis_tag_id)
                for lane_id in self.lane_bots.keys():
                    if dis_tag_id in self.lane_bots[lane_id]:
                        self.lane_bots[lane_id].remove(dis_tag_id)
            except Exception as e:
                rospy.loginfo(f"an error happened in dealing disappear tags: {e}")
        self.bots_in_last_frame = current_bots_ids

        previous_inter_bots_ids = []
        for bots_id in self.intersection_bots_n_frames:
            previous_inter_bots_ids += bots_id
        previous_inter_bots_ids = np.array(previous_inter_bots_ids)
        new_appear_in_inter = np.setdiff1d(current_inter_bots_ids, np.unique(previous_inter_bots_ids))
        self.througput += len(new_appear_in_inter)
        self.intersection_bots_n_frames[:-1] = self.intersection_bots_n_frames[1:]
        self.intersection_bots_n_frames[-1] = list(current_inter_bots_ids)
        ############################################################

        # publish detections
        self._tag_pub.publish(tags_msg)
        # update healthy frequency metadata
        self._tag_pub.set_healthy_freq(self._img_sub.get_frequency())
        self._img_pub.set_healthy_freq(self._img_sub.get_frequency())

        ######################################################################
        # publish the observation times for cars
        bots_obs_time_list = SegmentList()
        bots_obs_time_list.header.stamp = msg.header.stamp
        bots_obs_time_list.segments.extend(self._to_obs_times_msg(self.bots_obs_times))
        # rospy.loginfo(bots_obs_time_list)
        self.pub_bots_obs_times.publish(bots_obs_time_list)

        # publish the lane bots info, including the centerpoints of apriltag and stop line
        lane_bots_list = SegmentList()
        lane_bots_list.header.stamp = msg.header.stamp
        for lane_id, bot_ids in self.lane_bots.items():
            lane_bots_list.segments.extend(self._to_lane_bots_msg(lane_id, bot_ids, current_bots_centers))
        self.pub_lane_bots.publish(lane_bots_list)
        ######################################################################

        # render visualization (if needed)
        if self._img_pub.anybody_listening() and not self._renderer_busy:
            self._renderer_busy = True
            Thread(target=self._render_detections, args=(msg, img, tags, num_bots)).start()

    ########################################################
    # function to construct traffic flow data
    def _to_obs_times_msg(self, dict_obs_times):
        bots_obs_times_msg = []
        for key, value in dict_obs_times.items():
            car_data = Segment()
            car_data.normal.x = key # car_data.normal.x = car's tag id
            car_data.normal.y = value # car_data.normal.y = the observed time of the car
            bots_obs_times_msg.append(car_data)
        return bots_obs_times_msg

    def _to_lane_bots_msg(self, lane_id, bots_ids, current_bots_centers):
        lane_bots_msg = []
        for bot_id in bots_ids:
            lane_bot = Segment()
            lane_bot.color = int(lane_id) # lane_bot.color = the index of the lane
            lane_bot.normal.x = bot_id # lane_bot.normal.x = the tag of a bot
            lane_bot.normal.y = int()
            lane_bot.points[0].x = current_bots_centers[bot_id][0] # get the centerpoint of this tag
            lane_bot.points[0].y = current_bots_centers[bot_id][1] # get the centerpoint of this tag
            lane_bot.points[1].x = self.stoplines_centers[int(lane_id)][0] # get the centerpoint of stop line of the lane
            lane_bot.points[1].y = self.stoplines_centers[int(lane_id)][1] # get the centerpoint of stop line of the lane
            lane_bots_msg.append(lane_bot)
        return lane_bots_msg
    ########################################################

    def cbTrafficLightPattern(self, msg):
        self.TrafficLightPattern = str(int(min(msg.normal.x, msg.normal.y)))+str(int(max(msg.normal.x, msg.normal.y)))
        self.log("Traffic Light Pattern received: {}".format(self.TrafficLightPattern))

    def _img_cb(self, msg):
        # make sure we have received camera info
        if self._camera_parameters is None:
            return
        # make sure we have a rectification map available
        if self._mapx is None or self._mapy is None:
            return
        # make sure somebody wants this
        if (not self._img_pub.anybody_listening()) and (not self._tag_pub.anybody_listening()):
            return
        # make sure this is a good time to detect (always keep this as last check)
        if not self._detection_reminder.is_time(frequency=self.detection_freq.value):
            return
        # make sure we are still running
        if self.is_shutdown:
            return
        # ---
        # find the first available worker (if any)
        for i in range(self.ndetectors):
            if self._tasks[i] is None or self._tasks[i].done():
                # submit this image to the pool
                self._tasks[i] = self._workers.submit(self._detect, i, msg)
                break

    def _render_detections(self, msg, img, detections, num_bots):
        with self.profiler('/publishs_image'):
            # get a color buffer from the BW image
            img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
            # draw each tag
            for detection in detections:
                for idx in range(len(detection.corners)):
                    cv2.line(
                        img,
                        tuple(detection.corners[idx - 1, :].astype(int)),
                        tuple(detection.corners[idx, :].astype(int)),
                        (0, 255, 0)
                    )
                # draw the tag ID
                cv2.putText(
                    img,
                    str(detection.tag_id),
                    org=(
                        detection.corners[0, 0].astype(int) + 10,
                        detection.corners[0, 1].astype(int) + 10
                    ),
                    fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                    fontScale=0.8,
                    color=(0, 0, 255)
                )

                #################################################
                # print observed time for each bot
                try:
                    cv2.putText(
                        img,
                        str(round(self.bots_obs_times[detection.tag_id], 1)),
                        org=(
                            detection.corners[0, 0].astype(int) + 10,
                            detection.corners[0, 1].astype(int) + 30
                        ),
                        fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.8, thickness=2, color=(0, 0, 255)
                    )
                except Exception as e:
                    pass
                #################################################

            ##############################################
            # draw different lanes
            self.draw_lanes(img)
            # draw traffic light
            self.draw_lights(img)

            try:
                current_timeee = rospy.Time.now().to_sec()
                passed_timeee = current_timeee - self.start_s

                cv2.putText(
                    img, "Total time: " + str(round(passed_timeee, 1)),
                    org=(30, 120 + 17 * 10), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1.2, thickness=3, color=(0, 0, 255)
                )

                cv2.putText(
                    img, "Total waiting time: " + str(round(self.total_waiting_time, 1)),
                    org=(30, 120 + 17 * 12), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1.2, thickness=3, color=(0, 0, 255)
                )

                rospy.loginfo(f"Passed time: {passed_timeee}, self.start_s: {self.start_s}")

                if passed_timeee <= self.limited_sec:
                    filename = os.path.join(self.output_dir, f"frame_{passed_timeee}.jpg")
                    try:
                        cv2.imwrite(filename, img)
                        rospy.loginfo(f"Frame {passed_timeee} written to {filename}")
                    except Exception as e:
                        rospy.logerr("Failed to save frame: {}".format(e))
            except Exception as e:
                rospy.loginfo("Failed to write total time or waiting time on image")
            ##############################################

            # pack image into a message
            img_msg = CompressedImage()
            img_msg.header.stamp = msg.header.stamp
            img_msg.header.frame_id = msg.header.frame_id
            img_msg.format = 'jpeg'
            img_msg.data = self._jpeg.encode(img)
        # ---
        self._img_pub.publish(img_msg)
        self._renderer_busy = False

    def draw_lanes(self, img):
        lane_idx = 0
        for hor_line in self.hor_lines:
            cv2.line(img, hor_line[0], hor_line[1], (0, 255, 0), thickness=2)
            if lane_idx == 0: pass
            else:
                cv2.putText(img, f"Lane {0+lane_idx}", org=(hor_line[0][0]+20, hor_line[0][1]-80), fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                            fontScale=1.2, thickness=3, color=(0, 255, 0))
                cv2.putText(img, f"Lane {7-lane_idx}", org=(hor_line[1][0]-140, hor_line[1][1]-80), fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                            fontScale=1.2, thickness=3, color=(0, 255, 0))
            lane_idx += 1
        lane_idx = 0
        for ver_line in self.ver_lines:
            cv2.line(img, ver_line[0], ver_line[1], (0, 255, 0), thickness=2)
            if lane_idx == 2: pass
            else:
                cv2.putText(img, f"Lane {8-lane_idx}", org=(ver_line[0][0]+20, ver_line[0][1]+50), fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                            fontScale=1.2, thickness=3, color=(0, 255, 0))
                cv2.putText(img, f"Lane {3+lane_idx}", org=(ver_line[1][0]+20, ver_line[1][1]-30), fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                            fontScale=1.2, thickness=3, color=(0, 255, 0))
            lane_idx += 1

        for lane_id in self.lane_bots.keys():
            cv2.putText(img, f"Lane {lane_id}: "+str(self.lane_bots[lane_id])+f", total {len(self.lane_bots[lane_id])} vehicles",
                        org=(30, 60+17*lane_id),
                        fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                        fontScale=1.2, thickness=3, color=(0, 255, 0))

        cv2.putText(img, f"throughput: {self.througput}",
                        org=(30, 120+17*8),
                        fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                        fontScale=1.2, thickness=3, color=(0, 0, 255))

    def draw_lights(self, img):
        cv2.rectangle(img,
                      (int(9*self.total_col/12), int(1*self.total_row/12)),
                      (int(11*self.total_col/12), int(3*self.total_row/12)), (255,255,255), thickness=2)
        if self.TrafficLightPattern=='13':
            cv2.circle(img, (int(10 * self.total_col / 12), int(1 * self.total_row / 12)), 8, (0, 0, 255), thickness=-1)
            cv2.circle(img, (int(10 * self.total_col / 12), int(3 * self.total_row / 12)), 8, (0, 0, 255), thickness=-1)
            cv2.circle(img, (int(9 * self.total_col / 12), int(2 * self.total_row / 12)), 8, (0, 255, 0), thickness=-1)
            cv2.circle(img, (int(11 * self.total_col / 12), int(2 * self.total_row / 12)), 8, (0, 255, 0), thickness=-1)
        elif self.TrafficLightPattern=='02':
            cv2.circle(img, (int(10 * self.total_col / 12), int(1 * self.total_row / 12)), 8, (0, 255, 0), thickness=-1)
            cv2.circle(img, (int(10 * self.total_col / 12), int(3 * self.total_row / 12)), 8, (0, 255, 0), thickness=-1)
            cv2.circle(img, (int(9 * self.total_col / 12), int(2 * self.total_row / 12)), 8, (0, 0, 255), thickness=-1)
            cv2.circle(img, (int(11 * self.total_col / 12), int(2 * self.total_row / 12)), 8, (0, 0, 255), thickness=-1)


def _matrix_to_quaternion(r):
    T = np.array((
        (0, 0, 0, 0),
        (0, 0, 0, 0),
        (0, 0, 0, 0),
        (0, 0, 0, 1)
    ), dtype=np.float64)
    T[0:3, 0:3] = r
    return tf.transformations.quaternion_from_matrix(T)


if __name__ == '__main__':
    node = AprilTagDetector()
    # spin forever
    rospy.spin()
