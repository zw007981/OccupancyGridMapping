#!/usr/bin/env python
import rospy
import math
from control import *


if __name__ == "__main__":
    rospy.init_node('test_control')
    pub_cmd = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    # Distance for each step.
    distance = 5.0
    total_steps = 20
    step = 0
    # Resolution for the map.
    resolution = 0.02
    size = int(20/resolution)
    map_maker = MapMaker(origin=-10, resolution=resolution, size=size)
    robot = Robot(pub_cmd)
    robot.get_pose()
    next_move = map_maker.process_scan(robot)
    [x_initial, y_initial] = [robot.xNow, robot.yNow]
    robot.x_trajectory.append(x_initial)
    robot.y_trajectory.append(y_initial)
    while step < total_steps:
        step += 1
        print("{}/{}, current position: [{:.1f},{:.1f}], direction:{}.".format(
            step, total_steps, robot.xNow, robot.yNow, next_move))
        robot.rotate_to_angle(next_move)
        robot.move_forward(distance, map_maker)
        map_maker.detect_loop_closure(robot)
        if map_maker.flagLoopDetected == 1:
            robot.follow_loop(map_maker)
            step = total_steps
        else:
            next_move = map_maker.process_scan(robot)
            rospy.sleep(0.3)
    map_maker.visualize_map()
