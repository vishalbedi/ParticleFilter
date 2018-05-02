#!/usr/bin/env python

import math
import random
import rospy
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

LINEAR_VELOCITY = 0.2
ANGULAR_VELOCITY = 0.4
TOLERANCE = 0.3
ROBOT_RADIUS = 0.22
OBSTACLE_THRESHOLD = 0.78
EXIT_STATUS_ERROR = 1
EXIT_STATUS_OK = 0


class SafeWander:
    def __init__(self, goals):
        # rospy.init_node('traveler', anonymous=True)
        self.vel_publisher = rospy.Publisher('/r1/cmd_vel', Twist, queue_size=10)
        # Hold position and quaternion of robot
        self.pos = Pose()
        self.theta = 0
        self.obstacle_found = False
        self.obstacle_circumventing = False
        self.start = (0, 0)
        self.goal = None
        self.mline = None
        self.curr_line = None
        self.sonar_data = []
        self.vel_msg = Twist()
        self.rate = rospy.Rate(10)
        self.islocalized = False
        self.goals = goals

    def odom_callback(self, odom):  # will continuously update current position
        """
		Callback function for the odometry subscriber, which will continuously update the 
		current position of the robot.
		@arg 	odom 	the odometry data of the robot
		"""
        self.pos = odom.pose.pose
        self.theta = 2 * math.atan2(self.pos.orientation.z, self.pos.orientation.w)
        self.curr_line = self.slope((self.pos.position.x, self.pos.position.y), self.goal)

    def laser_callback(self, laser_data):
        """
		Callback function for the laser subscriber, which will continuously update the 
		current status of whether an obstacle is detected or not
		"""
        # check if laser_data.ranges[321:350] is not infinity
        region = laser_data.ranges[321:350]
        m = float('inf')
        for d in region:
            if d < OBSTACLE_THRESHOLD and not self.obstacle_circumventing:
                self.obstacle_found = True
                self.stop()
                print("Obstacle Detected!")
                break

    def sonar_callback(self, sonar_data):
        """
		Callback function for the sonar subscriber, which will continuously update the 
		current status of sonar obstacle detection data
		@arg 	sonar_data 	LaserScan object with a list "range" of range 0-682
		"""
        self.sonar_data = sonar_data.ranges

    def slope(self, p1, p2):
        """
        Method to calculate the slope of a line segment formed by two given points
        @arg 	p1 	first point of line segment
        @arg 	p2 	second point of line segment
        @returns 	slope of line segment
        """
        try:
            delta_y = p2[1] - p1[1]
            delta_x = p2[0] - p1[0]
            return delta_y / delta_x if delta_x != 0 else float('inf')
        except:
            return 1

    def euclidean_distance(self):
        """
		Method to compute distance from current position to the goal
		@returns 	euclidean distance from current point to goal
		"""
        return math.sqrt(
            math.pow((self.goal[0] - self.pos.position.x), 2) + math.pow((self.goal[1] - self.pos.position.y), 2))

    def angular_difference(self):
        """
		Method to compute the absolute difference between the desired angle and the current 
		angle of the robot.
		@returns 	difference between current and desired angles
		"""
        return math.atan2(self.goal[1] - self.pos.position.y, self.goal[0] - self.pos.position.x) - self.theta

    def stop(self):
        """
		Method to bring robot to a halt by publishing linear and angular velocities of zero.
		"""
        self.vel_msg.linear.x = 0
        self.vel_msg.angular.z = 0
        self.vel_publisher.publish(self.vel_msg)

    def bug(self):
        """
		Method to climb the obstacle by going around it
		"""
        self.obstacle_circumventing = True

        min_sonar = min(self.sonar_data)
        LEFT90 = 600
        # rotate in place until sonar on left of robot +90 degrees is along tangent of obstacle surface
        while abs(self.sonar_data[LEFT90] - min_sonar) > 0.05:
            self.vel_msg.angular.z = -0.1
            self.vel_publisher.publish(self.vel_msg)
        self.stop()

        slope_threshold = 0.005

        while abs(self.curr_line - self.mline) > 0.1:
            # print(abs(self.curr_line - self.mline))
            self.vel_msg.linear.x = 0.1
            if self.sonar_data[LEFT90] < OBSTACLE_THRESHOLD - 0.2:  # too close to obstacle, rotate away
                LEFT90 = 600
                self.vel_msg.angular.z = -0.1
            elif self.sonar_data[LEFT90] > OBSTACLE_THRESHOLD + 0.2:  # too far, move closer
                LEFT90 = 550
                self.vel_msg.angular.z = 0.3
            else:
                self.vel_msg.angular.z = 0.0

            self.vel_publisher.publish(self.vel_msg)

        self.stop()

        self.start = (self.pos.position.x, self.pos.position.y)
        self.obstacle_found = False
        self.obstacle_circumventing = False
        self.go()

        print("Obstacle avoided")

    def go(self):
        """
		Method to go to goal specified irrespective of the current position and orientation 
		of the robot.
		"""

        # keep traveling until distance from current position to goal is greater than 0
        while self.euclidean_distance() > TOLERANCE and not self.obstacle_found and not self.islocalized:
            # print("distance from goal " + str(self.goal) + ": ", self.euclidean_distance())
            # set linear velocity
            self.vel_msg.linear.x = min(LINEAR_VELOCITY,
                                        LINEAR_VELOCITY * self.euclidean_distance())
            self.vel_msg.linear.y = 0
            self.vel_msg.linear.z = 0
            # set angular velocity
            self.vel_msg.angular.x = 0
            self.vel_msg.angular.y = 0
            self.vel_msg.angular.z = min(ANGULAR_VELOCITY,
                                         ANGULAR_VELOCITY * self.angular_difference())
            # publish velocity
            self.vel_publisher.publish(self.vel_msg)
            rospy.sleep(0.01)
            self.rate.sleep()

        if self.obstacle_found:
            self.stop()
            self.bug()
        else:
            self.stop()
            self.start = self.goal

    def travel(self, event):
        for goal in self.goals:
            self.start = (self.pos.position.x, self.pos.position.y)
            self.mline = self.slope(self.start, goal)
            self.goal = goal
            self.go()
            if self.islocalized:
                break


if __name__ == '__main__':
    coordinates = []
    for i in range(20):
        x = random.randint(-10, 10)
        y = random.randint(-10, 10)
        p = (x, y)
        coordinates.append(p)

    Robot = SafeWander(coordinates)
    Robot.travel()
