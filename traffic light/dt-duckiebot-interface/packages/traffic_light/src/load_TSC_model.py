import numpy as np

def find_longest_time_lane(lane_bots, bots_obs_times):
    longest_time_lane = {2:0, 4:0, 6:0, 8:0}

    for lane_id, bots_ids in lane_bots.items():
        for bot_id in bots_ids:
            if bots_obs_times[bot_id] > longest_time_lane[lane_id]:
                longest_time_lane[lane_id] = bots_obs_times[bot_id]

    return list(longest_time_lane.values())

def find_vehicle_num_lane(lane_bots):
    return [len(lane_bots[2]), len(lane_bots[4]), len(lane_bots[6]), len(lane_bots[8])]

def change_direction_rule(lane_bots, bots_obs_times):

    # always regard as a 4-way intersection
    # input of the rule-based policy
    # 1. list with length 4: [bot_with_longest_time_in_lane2, bot_with_longest_time_in_lane4, bot_with_longest_time_in_lane6, bot_with_longest_time_in_lane8]
    lane_times = find_longest_time_lane(lane_bots, bots_obs_times)
    # 2. list with length 4: [bots_number_in_lane2, bots_number_in_lane4, bots_number_in_lane6, bots_number_in_lane8]
    lane_nums = find_vehicle_num_lane(lane_bots)

    # traffic light pattern, this should be the output
    color_list = []




    ################################################################################
    # rule-based policy
    if (max(lane_times[0], lane_times[2]) + 2 * (lane_nums[0] + lane_nums[2]) >
        max(lane_times[1], lane_times[3]) + 2 * (lane_nums[1] + lane_nums[3])):
        color_list = [1,0,1,0]
    elif (max(lane_times[0], lane_times[2]) + 2 * (lane_nums[0] + lane_nums[2]) <
          max(lane_times[1], lane_times[3]) + 2 * (lane_nums[1] + lane_nums[3])):
        color_list = [0,1,0,1]
    else:
        # Change every 2 sec
        pass
    ################################################################################



    return color_list



if __name__ == '__main__':
    # dummy input
    # Traffic flow info
    # the bots in each lane. for example, lane 2 has bot with id:432 and 429. Lane 6 does not have any bot.
    lane_bots = {2: [432, 429], 4: [435], 6: [], 8: [428, 425, 436]}
    # total time a bot has been observed by the camera. For example, bot with id 432 has been observed for 3.5 sec.
    bots_obs_times = {432:3.5, 429:15.6, 435:4.8, 428:7.0, 425:25.9, 436:10.3}
    color_list = change_direction_rule(lane_bots, bots_obs_times)

    print(color_list)
