#!/usr/bin/env python3
import rospy

from duckietown_msgs.msg import LEDPattern
from duckietown_msgs.srv import SetCustomLEDPattern

from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from duckietown.dtros.utils import apply_namespace
import os
from duckietown_msgs.msg import SegmentList, Segment
# from std_msgs.msg import String


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
        self.bots_obs_times = {}  # total time a bot has been observed by the camera

        # Function mapping to LEDEmitterNode's `set_custom_pattern` service
        self.changePattern = rospy.ServiceProxy(
            apply_namespace('led_emitter_node/set_custom_pattern', ns_level=1),
            SetCustomLEDPattern
        )

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

        # Create the subscribers
        self.sub_lane_bots = rospy.Subscriber(f"/{self.veh}/apriltag_detector_node/detections/lane_bots", SegmentList, self.cbProcessLaneBots, queue_size=1)
        self.sub_bots_obs_times = rospy.Subscriber(f"/{self.veh}/apriltag_detector_node/detections/bots_obs_times", SegmentList, self.cbProcessBotsObsTimes, queue_size=1)

        self.log("Initialized.")

    def cbProcessLaneBots(self, data):
        self.lane_bots = {2: [], 4: [], 6: [], 8: []}
        for lane_bots in data.segments:
            # rospy.loginfo(f"lane: {lane_bots.color}, bot_tag_id: {int(lane_bots.normal.x)}")
            self.lane_bots[int(lane_bots.color)].append(int(lane_bots.normal.x))
        # rospy.loginfo(self.lane_bots)

    def cbProcessBotsObsTimes(self, data):
        self.bots_obs_times = {}
        for bot_obs_time in data.segments:
            # rospy.loginfo(f"bot_tag_id: {bots_obs_times.color}, bots_obs_times: {bots_obs_times.normal.x}")
            self.bots_obs_times[int(bot_obs_time.normal.x)] = bot_obs_time.normal.y
        # rospy.loginfo(self.bots_obs_times)

    def change_direction_rule_num(self, event):
        green_LED_1 = 0
        green_LED_2 = 0

        num_car_2_6 = 0
        num_car_4_8 = 0

        if self._intersection_type == 3:  # 3-way intersection, lane 4 or lane 8 should never have traffic
            num_car_2_6 = len(self.lane_bots[2]) + len(self.lane_bots[6])
            num_car_4_8 = (len(self.lane_bots[4]) + len(self.lane_bots[8])) * 2
        else:  # 4-way intersection
            num_car_2_6 = len(self.lane_bots[2]) + len(self.lane_bots[6])
            num_car_4_8 = len(self.lane_bots[4]) + len(self.lane_bots[8])

        if num_car_2_6 > num_car_4_8:  # lane 4 or lane 8 should never have traffic
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
        # TODO
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
