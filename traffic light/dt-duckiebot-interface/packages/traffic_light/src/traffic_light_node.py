#!/usr/bin/env python3
import numpy as np
import rospy

from duckietown_msgs.msg import LEDPattern
from duckietown_msgs.srv import SetCustomLEDPattern

from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from duckietown.dtros.utils import apply_namespace
import os
from duckietown_msgs.msg import SegmentList, Segment
# from std_msgs.msg import String
##############################
# import onnxruntime as ort
# import numpy as np
##############################



class TrafficLightNode(DTROS):
    """Handles the LED patterns for the traffic lights.

    The class creates an object which handles the timers for managing
    a traffic light at an intersection. By default a 4-way intersection
    is assumed, this can be modified by the `~number_leds` configuration parameter.
    The actual execution of the protocol, and the communication with the driver is done
    by :obj:`LEDEmitterNode`, so it must be running for the correct operation of this node.
    An additional protocol defines the length of the periods of green light
    and the order of the directions. All configuration parameters are dynamically configurable
    and can be updated via `rosparam set`.

    Configuration:
        ~number_leds (:obj:`int`): Number of LEDs, should be 3 or 4 depending on the
            intersection type, default is 4
        ~activation_order (:obj:`list` of :obj:`int`): The order of activation of the
            LEDs, default is [0,1,2,3]
        ~green_time (:obj:`float`): Duration of the green signal in seconds, default is 5
        ~all_red_time (:obj:`float`): Duration of a signal when all direction are red (waiting
            for the Duckiebots to clear the intersection) in seconds, default is 4
        ~frequency (:obj:`float`): The blinking frequency of the green signal, default is 7.8

    """

    # def __init__(self, node_name, intersection, mode, cycle_duration):
    def __init__(self, node_name):

        # Initialize the DTROS parent class
        super(TrafficLightNode, self).__init__(
            node_name=node_name,
            node_type=NodeType.COMMUNICATION
        )

        # Import protocols
        self._number_leds = rospy.get_param("~number_leds")
        self._activation_order = rospy.get_param("~activation_order")
        self._green_time = rospy.get_param("~green_time")
        self._all_red_time = rospy.get_param("~all_red_time")
        self._frequency = rospy.get_param("~frequency")
        # self._intersection_type = intersection
        self._intersection_type = rospy.get_param("~intersection")

        self.green_idx = 0
        # self._mode = mode
        self._mode = rospy.get_param("~mode")

        # self.cycle_duration = self._green_time + self._all_red_time
        # self.cycle_duration = cycle_duration
        self.cycle_duration = rospy.get_param("~cycle_duration")
        self.veh = os.environ['VEHICLE_NAME']

        # Create the color mask
        self.color_mask = [0]*5
        self.color_mask[0:self._number_leds] = [1]*self._number_leds

        # Traffic flow info
        self.lane_bots = {2: [], 4: [], 6: [], 8: []}
        self.lane_bots_last = {2: [], 4: [], 6: [], 8: []} # the bots in each lane captured in the last frame
        self.bots_lane = {} # {430:2, 432:2, 429:8}
        self.bots_lane_last = {} # the bots in which lane captured in the last frame
        self.bots_obs_times = {} # total time a bot has been observed by the camera {430:5.7, 432:10.3, 429:25.9}

        # Function mapping to LEDEmitterNode's `set_custom_pattern` service
        self.changePattern = rospy.ServiceProxy(
            apply_namespace('led_emitter_node/set_custom_pattern', ns_level=1),
            SetCustomLEDPattern
        )

        #########################################################
        # possible car tags and name
        self.fleets = {429:'xiao0o0o', 430:'madmax', 431:'starborn', 432:'bob', 433:'dlc', 434:'beefboss'}
        # self.fleets = {429:'xiao0o0o', 430:'madmax'}
        # build publishers for each bot
        # self.pubs_bots_traffic = {}
        # for key, value in self.fleets.items():
        #     self.pubs_bots_traffic[key] = rospy.Publisher(f'/{self.veh}/{value}/stop_or_go', Segment, queue_size=1, dt_topic_type=TopicType.PERCEPTION)
        self.pubs_bots_traffic = rospy.Publisher(f'/{self.veh}/stop_or_go', Segment, queue_size=1, dt_topic_type=TopicType.PERCEPTION)

        # the distance for each observed bot to the center of the stop line {432:12.333, 431:3.564}
        self.bots_to_stopline = {}
        self.light_lane = {2: 'red', 4: 'red', 6: 'red', 8: 'red'} # the traffic light for each lane
        self.light_lane_last = {2: 'red', 4: 'red', 6: 'red', 8: 'red'}  # the traffic light for each lane last time
        self.thresh_stopline_distance = 600 # how far (pixels) the bot will stop at the stop line

        # bots status at current frame
        self.bot_is_sent_stop = {} # {429:True, 430:False...}
        #########################################################

        # Start a timer that will regularly call a method that changes
        # the direction that get green light
        if self._mode == "rule_num":
            self.traffic_cycle = rospy.Timer(
                rospy.Duration(self.cycle_duration),
                self.change_direction_rule_num
            )
        elif self._mode == "rule_time":
            self.traffic_cycle = rospy.Timer(
                rospy.Duration(self.cycle_duration),
                self.change_direction_rule_time
            )
        elif self._mode == "TSC_model":
            self.traffic_cycle = rospy.Timer(
                rospy.Duration(self.cycle_duration),
                self.change_direction_TSC_model
            )
        else:
            self.traffic_cycle = rospy.Timer(
                rospy.Duration(self.cycle_duration),
                self.change_direction
            )


        self.pub_light = rospy.Publisher('~traffic_light_LED', Segment, queue_size=1, dt_topic_type=TopicType.PERCEPTION)
        self.pub_distance = rospy.Publisher(f"/{self.veh}/distance", Segment, queue_size=1, dt_topic_type=TopicType.PERCEPTION)

        # Create the subscribers
        self.sub_lane_bots = rospy.Subscriber(f"/{self.veh}/apriltag_detector_node/detections/lane_bots", SegmentList, self.cbProcessLaneBots, queue_size=1)
        self.sub_bots_obs_times = rospy.Subscriber(f"/{self.veh}/apriltag_detector_node/detections/bots_obs_times", SegmentList, self.cbProcessBotsObsTimes, queue_size=1)

        self.log("Initialized.")

    def cbProcessLaneBots(self, data):
        self.lane_bots_last = self.lane_bots
        lane_bots_current = {2: [], 4: [], 6: [], 8: []}
        self.bots_lane_last = self.bots_lane
        bots_lane_current = {}
        bots_to_stopline_current = {}
        for lane_bot in data.segments:
            # rospy.loginfo(f"lane: {lane_bot.color}, bot_tag_id: {int(lane_bot.normal.x)}")
            lane_id = int(lane_bot.color)
            bot_id = int(lane_bot.normal.x)
            lane_bots_current[lane_id].append(bot_id)
            bots_lane_current[bot_id] = lane_id
            bots_to_stopline_current[bot_id] \
                = ((lane_bot.points[0].x - lane_bot.points[1].x) ** 2 + (lane_bot.points[0].y - lane_bot.points[1].y) ** 2) ** 0.5

        self.lane_bots = lane_bots_current
        self.bots_lane = bots_lane_current
        self.bots_to_stopline = bots_to_stopline_current
        # rospy.loginfo(self.lane_bots)

        # find the disappear bots for each lane
        disappear_bots_each_lane = {} # {2:[], 4:[], 6:[], 8:[]}
        # find the new appear bots for each lane
        new_appear_bots_each_lane = {} # {2:[], 4:[], 6:[], 8:[]}
        for lane_id, bots_ids in self.lane_bots.items():
            disappear_bots = np.setdiff1d(self.lane_bots_last[lane_id], self.lane_bots[lane_id])
            disappear_bots_each_lane[lane_id] = disappear_bots # the disappear bots ids in one lane
            for bot_id in disappear_bots:
                self.bot_is_sent_stop.pop(int(bot_id), None) # delete the bots in is sent stop
            new_appear_bots = np.setdiff1d(self.lane_bots[lane_id], self.lane_bots_last[lane_id])
            new_appear_bots_each_lane[lane_id] = new_appear_bots
            for bot_id in new_appear_bots:
                self.bot_is_sent_stop[bot_id] = False # the new observed bots are not sent stop

    def cbProcessBotsObsTimes(self, data):
        self.bots_obs_times = {}
        for bot_obs_time in data.segments:
            # rospy.loginfo(f"bot_tag_id: {bots_obs_times.color}, bots_obs_times: {bots_obs_times.normal.x}")
            self.bots_obs_times[int(bot_obs_time.normal.x)] = bot_obs_time.normal.y
        # rospy.loginfo(self.bots_obs_times)

    def change_direction_rule_num(self, event):
        green_LED_1 = 0
        green_LED_2 = 0

        # always regard as a 4-way intersection
        num_car_2_6 = len(self.lane_bots[2]) + len(self.lane_bots[6])
        num_car_4_8 = len(self.lane_bots[4]) + len(self.lane_bots[8])

        if num_car_2_6 > num_car_4_8:
            green_LED_1 = 1
            green_LED_2 = 3
        elif num_car_2_6 < num_car_4_8:
            green_LED_1 = 0
            green_LED_2 = 2
        else:
            self.green_idx = (self.green_idx + 1) % self._number_leds
            green_idx_1 = self.green_idx
            green_idx_2 = (green_idx_1 + 2) % self._number_leds
            green_LED_1 = self._activation_order[green_idx_1]
            green_LED_2 = self._activation_order[green_idx_2]

        frequency_mask = [0] * 5
        # Create Protocol (we fake 5 LEDs, but the last will not be updated)
        color_list = ['red'] * 5
        color_list[green_LED_1] = 'green'
        color_list[green_LED_2] = 'green'
        # Build message
        pattern_msg = LEDPattern()
        pattern_msg.color_list = self.to_led_order(color_list)
        pattern_msg.color_mask = self.color_mask
        pattern_msg.frequency = self._frequency
        pattern_msg.frequency_mask = self.to_led_order(frequency_mask)

        self.changePattern(pattern_msg)

        traffic_light_msg = Segment()
        traffic_light_msg.normal.x = green_LED_1
        traffic_light_msg.normal.y = green_LED_2
        self.pub_light.publish(traffic_light_msg)

        green_lights_ids = np.array([green_LED_1, green_LED_2])
        self.send_msg_to_bot(green_lights_ids)

    def send_msg_to_bot(self, green_lights_ids):
        #################################################################
        self.light_lane_last = self.light_lane
        red_light = np.setdiff1d(np.array(list(range(4))), green_lights_ids)
        # light_id -> lane_id: 0->8, 1->2, 2->4, 3->6
        for light_id in green_lights_ids:
            if light_id > 0:
                self.light_lane[(int(light_id) * 2)] = "green"
            else:
                self.light_lane[8] = "green"
        for light_id in red_light:
            if light_id > 0:
                self.light_lane[(int(light_id) * 2)] = "red"
            else:
                self.light_lane[8] = "red"
        # self._intersection_type
        for bot_id, bot_to_stopline in self.bots_to_stopline.items():
            # 对于每一辆被观察到的车，如果车道红灯，够近，没被发过要停，则发停止的指令，并修改状态为被发送停止信号
            distance_msg = Segment()
            distance_msg.normal.x = bot_id
            distance_msg.normal.y = bot_to_stopline
            distance_msg.points[0].x = bot_id
            distance_msg.points[0].y = ord(self.light_lane[int(self.bots_lane[bot_id])][0])
            distance_msg.points[0].z = int(self.bot_is_sent_stop[bot_id])
            distance_msg.points[1].x = int(self.bots_lane[bot_id])
            self.pub_distance.publish(distance_msg)
            if (self.light_lane[int(self.bots_lane[bot_id])] == 'red') and (
                    bot_to_stopline <= self.thresh_stopline_distance) and (
                    not self.bot_is_sent_stop[bot_id]):
                try:
                    self.bot_is_sent_stop[bot_id] = True
                    self.publish_stop_msg(bot_id) ###################publish stop message to the right bot
                except Exception as e:
                    rospy.loginfo(e)
            # 对于每一辆被观察的车，如果被发送了停止信号，灯变绿， 则发行走的指令，并修改状态为未被发送停止信号
            if (self.bot_is_sent_stop[bot_id]) and (self.light_lane[int(self.bots_lane[bot_id])] == 'green'):
                try:
                    self.bot_is_sent_stop[bot_id] = False
                    self.publish_go_msg(bot_id)
                except Exception as e:
                    rospy.loginfo(e)
        #################################################################

    def publish_stop_msg(self, bot_id):
        stop_msg = Segment()
        stop_msg.color = 1 # color==1, stop
        stop_msg.normal.x = bot_id # the id of bot is in stop_msg.normal.x
        # rospy.loginfo(stop_msg)
        self.pubs_bots_traffic.publish(stop_msg)

    def publish_go_msg(self, bot_id):
        go_msg = Segment()
        go_msg.color = 0 # color==0, go
        go_msg.normal.x = bot_id # the id of bot is in go_msg.normal.x

        # determine what action can be done (left, straight, right)
        go_msg.points[0].x = 1  # go_msg.points[0].x is whether the bot can turn left
        go_msg.points[0].y = 1  # go_msg.points[0].y is whether the bot can go straight
        go_msg.points[0].z = 1  # go_msg.points[0].z is whether the bot can turn right

        if self._intersection_type == 3: # 3-way intersection, the bot cannot in lane4
            if self.bots_lane[bot_id] == 2: # if the bot is in lane2
                go_msg.points[0].z = 0 # the bot cannot turn right
            elif self.bots_lane[bot_id] == 6: # if the bot is in lane6
                go_msg.points[0].x = 0  # the bot cannot turn left
            elif self.bots_lane[bot_id] == 8: # if the bot is in lane8
                go_msg.points[0].y = 0  # the bot cannot go straight

        elif self._intersection_type == 4: # 4-way intersection
            pass # the bot can turn however it likes

        # rospy.loginfo(go_msg)
        self.pubs_bots_traffic.publish(go_msg)


    def change_direction_rule_time(self, event):
        green_LED_1 = 0
        green_LED_2 = 0

        longest_time = -10.
        longest_car = 999.
        longest_lane = 888

        if len(self.bots_obs_times.keys()) > 0: # there is some cars observed by the trafficlight
            for car_tag_id, obs_time in self.bots_obs_times.items():
                if obs_time > longest_time:
                    longest_time = obs_time
                    longest_car = car_tag_id
            for lane_id, lane_bots in self.lane_bots.items():
                if longest_car in lane_bots:
                    longest_lane = lane_id
                    break
            if longest_lane == 2 or longest_lane == 6:
                green_LED_1 = 1
                green_LED_2 = 3
            else:
                green_LED_1 = 0
                green_LED_2 = 2
        else:
            self.green_idx = (self.green_idx + 1) % self._number_leds
            green_idx_1 = self.green_idx
            green_idx_2 = (green_idx_1 + 2) % self._number_leds
            green_LED_1 = self._activation_order[green_idx_1]
            green_LED_2 = self._activation_order[green_idx_2]

        frequency_mask = [0]*5
        # Create Protocol (we fake 5 LEDs, but the last will not be updated)
        color_list = ['red'] * 5
        color_list[green_LED_1] = 'green'
        color_list[green_LED_2] = 'green'
        # Build message
        pattern_msg = LEDPattern()
        pattern_msg.color_list = self.to_led_order(color_list)
        pattern_msg.color_mask = self.color_mask
        pattern_msg.frequency = self._frequency
        pattern_msg.frequency_mask = self.to_led_order(frequency_mask)

        self.changePattern(pattern_msg)

        traffic_light_msg = Segment()
        traffic_light_msg.normal.x = green_LED_1
        traffic_light_msg.normal.y = green_LED_2
        self.pub_light.publish(traffic_light_msg)

    def change_direction_TSC_model(self, event):
        # self.session = ort.InferenceSession("/code/catkin_ws/src/dt-duckiebot-interface/simple_nn.onnx")
        # input_data = np.random.rand(1, 4).astype(np.float32)
        # outputs = self.session.run(None, {"input": input_data})
        # rospy.loginfo(f"ONNX Model Output: {outputs}")
        pass

    def change_direction(self, event):
        """Callback changing direction of green light.

            The callback iterates through the LEDs of a traffic light and
            generates an LEDPattern message accordingly. The green light blinks
            at the defined frequency, while the red light stays static. After
            the green light blinks according to protocol, there is a dead zone
            in which all directions are on red. The message is published as a
            special pattern to the led_emitter_node.
        """
        # Move to next light in list
        # self.green_idx = (self.green_idx + 1) % self._number_leds
        # green_LED = self._activation_order[self.green_idx]

        ##################################
        self.green_idx = (self.green_idx + 1) % self._number_leds
        green_idx_1 = self.green_idx
        green_idx_2 = (green_idx_1 + 2) % self._number_leds
        green_LED_1 = self._activation_order[green_idx_1]
        green_LED_2 = self._activation_order[green_idx_2]
        # green_LED_1 = 0
        # green_LED_2 = 2
        ##################################

        # # Only blink the green LED
        # frequency_mask = [0]*5
        # frequency_mask[green_LED] = 1

        ########################################
        # red and green LED do not blink
        frequency_mask = [0] * 5
        ########################################

        # Create Protocol (we fake 5 LEDs, but the last will not be updated)
        color_list = ['red'] * 5
        # color_list[green_LED] = 'green'

        ##############################################
        color_list[green_LED_1] = 'green'
        color_list[green_LED_2] = 'green'
        ##############################################

        # Build message
        pattern_msg = LEDPattern()
        pattern_msg.color_list = self.to_led_order(color_list)
        pattern_msg.color_mask = self.color_mask
        pattern_msg.frequency = self._frequency
        pattern_msg.frequency_mask = self.to_led_order(frequency_mask)

        self.changePattern(pattern_msg)

        #     traffic_light_msg = String()
        #     traffic_light_msg.data = f"{green_LED_1}{green_LED_2}"
        #     self.pub_light.publish(traffic_light_msg)

        # Keep the green light on
        rospy.sleep(self.cycle_duration + 2)

        # # Turn all on red for safety
        # pattern_msg.color_list = ['red'] * 5
        # pattern_msg.frequency = 0
        # self.changePattern(pattern_msg)

    @staticmethod
    def to_led_order(unordered_list):
        """Change ordering from successive (0,1,2,3,4) to the one expected by the led emitter (0,4,1,3,2)

            Args:
                unordered_list (:obj:`list`): List to be ordered.
            Returns:
                :obj: `list`: Permutated list of length ~number_leds.
        """
        ordering = [0, 4, 1, 3, 2]
        ordered_list = [unordered_list[i] for i in ordering]
        return ordered_list


if __name__ == '__main__':
    # Initialize the node
    # mode: "rule_num", "rule_time", "TSC_model"
    # traffic_light_node = TrafficLightNode(node_name='traffic_light', intersection=3, mode="rule_num", cycle_duration=2)
    traffic_light_node = TrafficLightNode(node_name='traffic_light')
    # Keep it spinning to keep the node alive
    rospy.spin()
