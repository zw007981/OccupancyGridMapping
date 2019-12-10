#!/usr/bin/env python
import sys
import time
import Queue
import rospy
import tf
import math as math
from threading import Thread
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from geometry import *
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle


# A class for robot control.
class Robot:
    def __init__(self, publisher):
        # Publish the twist information.
        self.pub = publisher
        # Subscribe the laser information.
        self.sub = rospy.Subscriber('scan', LaserScan, self.laser_callback)
        self.update_rate = 60
        self.angle_tolerance = 0.1
        self.pos_tolerance = 0.05
        self.flagProcessTf = 0
        # Give tf some time to fill its buffer.
        print("Waiting for the tf listener...")
        self.tfListener = tf.TransformListener()
        rospy.sleep(3)
        try:
            self.tfListener.waitForTransform(
                '/odom', '/base_footprint', rospy.Time(), rospy.Duration(1.0))
        except Exception:
            rospy.loginfo("Unable to find transform.")
            rospy.signal_shutdown('tf Exception.')
        # Pose get from tf.
        self.xNow = 0.0
        self.yNow = 0.0
        self.thetaNow = 0.0
        self.flagProcessTf = 0
        self.x_trajectory = []
        self.y_trajectory = []
        self.theta_trajectory = []
        print("Start exploring.")

    # Receive and process the information from the lidar.
    def laser_callback(self, msg):
        # Parmaters about the lidar.
        self.lidarRangeMin = msg.range_min
        self.lidarRangeMax = msg.range_max
        self.lidarAngleMin = msg.angle_min
        self.lidarAngleMax = msg.angle_max
        self.lidarAngleIncrement = msg.angle_increment
        # Laser information received.
        num = len(msg.ranges)
        self.lidarRanges = msg.ranges
        # The minimal distance between the robot and
        # the obastacle in front of it.
        # self.minRange = msg.ranges[int(0.5 * num)]
        self.minRange = min(msg.ranges[int(0.48*num):int(0.52*num)])

    # Get robot's pose from tf.
    def get_pose(self):
        while not rospy.is_shutdown():
            try:
                [trans, rot] = self.tfListener.lookupTransform(
                    '/odom', '/base_footprint', rospy.Time(0))
                self.flagProcessTf = 1
                break
            except Exception:
                rospy.loginfo('tf Exception.')
                self.flagProcessTf = 0
                continue
        if self.flagProcessTf == 1:
            euler = euler_from_quaternion(rot)
            self.xNow = trans[0]
            self.yNow = trans[1]
            self.thetaNow = euler[2]

    # Publish the twist information.
    def publish_twist(self, x_vel, angular_vel):
        twist = Twist()
        twist.linear.x = x_vel
        twist.angular.z = angular_vel
        self.pub.publish(twist)

    # Rotate to a specified angle in the robot's coordinate frame.
    def rotate_to_angle(self, angle):
        # Turn right.
        if angle == "R":
            angle = 1.5 * math.pi
        # Turn left.
        elif angle == "L":
            angle = 0.5 * math.pi
        # Turn back.
        elif angle == "B":
            angle = 1.0 * math.pi
        # Keep moving forward.
        elif angle == "F":
            angle = 0.0 * math.pi
        angular_speed = 0.5
        destination_angle = normalize_angle(self.thetaNow + angle)
        error = normalize_angle(destination_angle - self.thetaNow)
        # Prepare to rotate.
        rate = rospy.Rate(self.update_rate)
        while abs(error) > self.angle_tolerance:
            if rospy.is_shutdown():
                break
            self.get_pose()
            error = normalize_angle(destination_angle-self.thetaNow)
            self.publish_twist(0, angular_speed)
            rate.sleep()
        # Stop the robot.
        self.publish_twist(0.0, 0.0)
        rospy.sleep(0.4)

    # Move to the destination.
    def move_to_destination(self, x_destination, y_destination, map_maker):
        self.get_pose()
        [x_initial, y_initial] = [self.xNow, self.yNow]
        angle_desination = math.atan2(
            y_destination - y_initial, x_destination - x_initial)
        distance_desination = math.sqrt(
            (x_destination - x_initial)**2 + (y_destination - y_initial)**2)
        # Rotate.
        self.rotate_to_angle(angle_desination - self.thetaNow)
        # Move forward.
        self.move_forward(distance_desination, map_maker)

    # Just move forward.
    def move_forward(self, distance, map_maker):
        self.get_pose()
        [x_initial, y_initial] = [self.xNow, self.yNow]
        sum_distance = 0
        speed = 0.3
        # Prepare to move.
        rate = rospy.Rate(self.update_rate)
        while abs(sum_distance - distance) >= self.pos_tolerance:
            if rospy.is_shutdown() or self.minRange <= 1.0:
                break
            self.publish_twist(speed, 0.0)
            self.get_pose()
            sum_distance = math.sqrt(
                (self.xNow - x_initial)**2 + (self.yNow - y_initial)**2)
            map_maker.process_scan(self)
            rate.sleep()
        # Stop the robot and record its position.
        self.publish_twist(0.0, 0.0)
        rospy.sleep(0.4)
        self.x_trajectory.append(self.xNow)
        self.y_trajectory.append(self.yNow)
        self.theta_trajectory.append(self.thetaNow)

    # If there is a loop detected, follow previous path.
    def follow_loop(self, map_maker):
        if map_maker.flagLoopDetected == 1:
            path = map_maker.loop_detected
            print("Start closing the loop.")
            for point in path:
                print("Move to [{:.1f}, {:.1f}].".format(point[0], point[1]))
                self.move_to_destination(point[0], point[1], map_maker)
                rospy.sleep(0.3)
        # Stop the robot.
        self.publish_twist(0.0, 0.0)


# A class for mapping.
class MapMaker:
    def __init__(self, origin, resolution, size):
        self.originX = origin
        self.originY = origin
        self.resolution = resolution
        self.sizeX = size
        self.sizeY = size
        self.unobserved = [0, 0, 0, 0]
        self.threshold1 = 4.0
        self.threshold2 = 10.0
        self.flagLoopDetected = 0
        # Data stored in cell can tell us whether there is a obastacle or not,
        # 0 means this is a free cell, 1 means this is a occupied cell.
        self.cell = OccupancyGrid()
        self.cell.data = [-1] * (self.sizeX * self.sizeY)
        # Lidar shows there is no obstacle in this cell.
        self.cell1 = OccupancyGrid()
        self.cell1.data = [0] * (self.sizeX * self.sizeY)
        # Lidar shows there is an obstacle in this cell.
        self.cell2 = OccupancyGrid()
        self.cell2.data = [0] * (self.sizeX * self.sizeY)
        # Measurement unavailable for this cell.
        self.cell3 = OccupancyGrid()
        self.cell3.data = [0] * (self.sizeX * self.sizeY)
        # Store detailed trajectory.
        self.x_trajectory = []
        self.y_trajectory = []

    def process_scan(self, robot):
        # Check cells in the map to decide the next movemet.
        def check_cells(self, gx2, gy2):
            num_covered = 0
            # Position of the robot.
            [gx1, gy1] = [self.x_trajectory[-1], self.y_trajectory[-1]]
            if abs(gx1 - gx2) >= abs(gy1 - gy2):
                direction = "y"
            else:
                direction = "x"
            # For front and back.
            if direction == "x":
                x_delta_list = range(-10, 10)
                y_delta_list = [-1, 0, 1]
            # For right and left.
            elif direction == "y":
                x_delta_list = [-1, 0, 1]
                y_delta_list = range(-10, 10)
            for x_delta in x_delta_list:
                for y_delta in y_delta_list:
                    [x_cell, y_cell] = [gx2 + x_delta, gy2 + y_delta]
                    index = to_index(x_cell, y_cell, self.sizeX)
                    num_covered1 = max(40, self.cell1.data[index])
                    num_covered2 = max(40, self.cell2.data[index])
                    num_covered += (num_covered1 + num_covered2)
            return num_covered
        # Record the robot's position for plotting.
        self.x_trajectory.append(robot.xNow)
        self.y_trajectory.append(robot.yNow)
        self.rays = []
        num_covered_back = 0
        num_covered_right = 0
        num_covered_front = 0
        num_covered_left = 0
        if robot.flagProcessTf == 1:
            # Range value is the distance from the start of the laser
            # to the obstacle, convert this to coordinate frame of
            # the robot then into the world coordinate system.
            num_rays = len(robot.lidarRanges)
            for i in range(num_rays):
                flag_inf = 0
                range_value = robot.lidarRanges[i]
                # If range_value == inf, we convert it to robot.lidarRangeMax.
                if range_value >= robot.lidarRangeMax:
                    flag_inf = 1
                    range_value = robot.lidarRangeMax
                angle_to_robot = robot.lidarAngleMin + i * robot.lidarAngleIncrement
                angle_to_world = angle_to_robot + robot.thetaNow
                x_world = range_value * math.cos(angle_to_world) + robot.xNow
                y_world = range_value * math.sin(angle_to_world) + robot.yNow
                self.rays.append([x_world, y_world])
                # Convert points in the wrold frame to grid cells.
                try:
                    [gx1, gy1] = to_grid(robot.xNow, robot.yNow, self.originX,
                                         self.originY, self.sizeX, self.sizeY, self.resolution)
                    [gx2, gy2] = to_grid(x_world, y_world, self.originX,
                                         self.originY, self.sizeX, self.sizeY, self.resolution)
                    flag_grid_convert = 1
                except Exception:
                    flag_grid_convert = 0
                if flag_grid_convert == 1:
                    # The set of points on the ray.
                    points = bresenham(gx1, gy1, gx2, gy2)
                    for j in range(len(points)):
                        x_point = points[j][0]
                        y_point = points[j][1]
                        index = to_index(x_point, y_point, self.sizeX)
                        # All points except the last one are in free cells.
                        if j < len(points) - 1:
                            # self.cell.data[index] = 0
                            self.cell1.data[index] += 1
                        else:
                            if flag_inf == 1:
                                self.cell1.data[index] += 1
                            else:
                                self.cell2.data[index] += 1
                # Back.
                if i == 0:
                    num_covered_back = check_cells(self, gx2, gy2)
                    if range_value <= 1.5:
                        num_covered_back = float("inf")
                # Right.
                elif i == int(0.25 * num_rays):
                    num_covered_right = check_cells(self, gx2, gy2)
                    if range_value <= 1.5:
                        num_covered_right = float("inf")
                # Front.
                elif i == int(0.5 * num_rays):
                    num_covered_front = check_cells(self, gx2, gy2)
                    if range_value <= 1.5:
                        num_covered_front = float("inf")
                # Left.
                elif i == int(0.75 * num_rays):
                    num_covered_left = check_cells(self, gx2, gy2)
                    if range_value <= 1.5:
                        num_covered_left = float("inf")
        self.num_covered_sets = [
            num_covered_back, num_covered_right, num_covered_front, num_covered_left]
        num_covered_minimal = min(self.num_covered_sets)
        if num_covered_front == num_covered_minimal:
            next_move = "F"
        elif num_covered_right == num_covered_minimal:
            next_move = "R"
        elif num_covered_left == num_covered_minimal:
            next_move = "L"
        else:
            next_move = "B"
        return next_move

    def detect_loop_closure(self, robot):
        def cal_distance1(point1, point2):
            distance = math.sqrt(
                (point1[0]-point2[0])**2 + (point1[1]-point2[1])**2)
            return distance

        def cal_distance2(index, robot):
            d2 = 0
            while index < len(robot.x_trajectory) - 2:
                point1 = [robot.x_trajectory[index], robot.y_trajectory[index]]
                point2 = [robot.x_trajectory[index+1],
                          robot.y_trajectory[index+1]]
                distance = cal_distance1(point1, point2)
                d2 += distance
                index += 1
            return d2
        self.loop_detected = []
        self.flagLoopDetected = 0
        point2 = [self.x_trajectory[-1], self.y_trajectory[-1]]
        for i in range(len(robot.x_trajectory)-1):
            point1 = [robot.x_trajectory[i], robot.y_trajectory[i]]
            d1 = cal_distance1(point1, point2)
            if d1 <= self.threshold1:
                d2 = cal_distance2(i, robot)
                if d2 >= self.threshold2:
                    # Go to poin1 then close the loop.
                    print("Loop closure detected.")
                    print(d1, d2)
                    self.flagLoopDetected = 1
            if self.flagLoopDetected == 1:
                break
        if self.flagLoopDetected == 1:
            x_trajectory_loop = robot.x_trajectory[i:]
            x_trajectory_loop.append(point1[0])
            y_trajectory_loop = robot.y_trajectory[i:]
            y_trajectory_loop.append(point1[1])
            for i in range(len(x_trajectory_loop)):
                x = x_trajectory_loop[i]
                y = y_trajectory_loop[i]
                point = [x, y]
                self.loop_detected.append(point)

    def visualize_map(self):
        print("Building the map...")
        plt.figure(figsize=(15, 15))
        ax = plt.gca()
        for i in range(len(self.cell.data)):
            if max(self.cell1.data[i], self.cell1.data[i]) <= 2:
                self.cell.data[i] = -1
            elif self.cell1.data[i] > self.cell2.data[i]:
                self.cell.data[i] = 0
            elif self.cell1.data[i] < self.cell2.data[i]:
                self.cell.data[i] = 1
        i = 0
        while i < len(self.cell.data):
            value = self.cell.data[i]
            [gx, gy] = [i % self.sizeX, i / self.sizeX]
            if value == 1:
                rec = Rectangle((gx, gy), width=1, height=1, facecolor='black')
                ax.add_patch(rec)
                i += 1
            elif value == 0:
                j = i
                while i / self.sizeX == (j+1) / self.sizeX and self.cell.data[j+1] == 0:
                    j += 1
                rec = Rectangle((gx, gy), width=j-i+1, height=1,
                                facecolor='black', alpha=0.2)
                ax.add_patch(rec)
                i = j + 1
            else:
                i += 1
        plt.plot([(x - self.originX) * 1.0 / self.resolution for x in self.x_trajectory],
                 [(y - self.originY) * 1.0 /
                  self.resolution for y in self.y_trajectory],
                 'r', label="trajectory")
        plt.legend()
        plt.axis('equal')
        plt.axis('off')
        plt.tight_layout()
        plt.savefig('./images/'+'map.png')
        plt.close()
        print("Done.")
