#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import random

class Turtlebot3Drive:
    def __init__(self):
        rospy.init_node('rambler_node', anonymous=True)
        
        # Initialize parameters
        self.escape_range = 30.0 * math.pi / 180.0  # Convert degrees to radians
        self.check_forward_dist = 0.7
        self.check_side_dist = 0.6
        self.tb3_pose = 0.0
        self.prev_tb3_pose = 0.0

        # Initialize publishers and subscribers
        cmd_vel_topic_name = rospy.get_param('~cmd_vel_topic_name', 'cmd_vel')
        self.cmd_vel_pub = rospy.Publisher(cmd_vel_topic_name, Twist, queue_size=10)
        self.laser_scan_sub = rospy.Subscriber('scan', LaserScan, self.laser_scan_callback)
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback)

        # Scan data initialization
        self.scan_data = [0, 0, 0]

        self.turtlebot3_state_num = 0  # Initial state

    def odom_callback(self, msg):
        siny = 2.0 * (msg.pose.pose.orientation.w * msg.pose.pose.orientation.z +
                      msg.pose.pose.orientation.x * msg.pose.pose.orientation.y)
        cosy = 1.0 - 2.0 * (msg.pose.pose.orientation.y * msg.pose.pose.orientation.y +
                            msg.pose.pose.orientation.z * msg.pose.pose.orientation.z)
        self.tb3_pose = math.atan2(siny, cosy)

    def laser_scan_callback(self, msg):
        scan_angle = [0, 30, 330]
        for i in range(3):
            if math.isinf(msg.ranges[scan_angle[i]]):
                self.scan_data[i] = msg.range_max
            else:
                self.scan_data[i] = msg.ranges[scan_angle[i]]

    def update_command_velocity(self, linear, angular):
        cmd_vel = Twist()
        cmd_vel.linear.x = linear
        cmd_vel.angular.z = angular
        self.cmd_vel_pub.publish(cmd_vel)
    
    def random_movement(self):
        cmd_vel = Twist()
        cmd_vel.linear.x = random.uniform(0.1, 0.5)  # Random linear speed between 0.1 and 0.5 m/s
        cmd_vel.angular.z = random.uniform(-1.0, 1.0)  # Random angular speed between -1.0 and 1.0 rad/s
        self.cmd_vel_pub.publish(cmd_vel)

    def control_loop(self):
        while not rospy.is_shutdown():
            if self.turtlebot3_state_num == 0:  # GET_TB3_DIRECTION
                if self.scan_data[0] > self.check_forward_dist:
                    if self.scan_data[1] < self.check_side_dist:
                        self.prev_tb3_pose = self.tb3_pose
                        self.turtlebot3_state_num = 2  # TB3_RIGHT_TURN
                    elif self.scan_data[2] < self.check_side_dist:
                        self.prev_tb3_pose = self.tb3_pose
                        self.turtlebot3_state_num = 3  # TB3_LEFT_TURN
                    else:
                        self.turtlebot3_state_num = 1
                if self.scan_data[0] < self.check_forward_dist:
                    self.prev_tb3_pose = self.tb3_pose
                    self.turtlebot3_state_num = 2  # TB3_RIGHT_TURN

            elif self.turtlebot3_state_num == 1:  # TB3_RANDOM_DRIVE
                self.random_movement()  # Forward with linear velocity
                self.turtlebot3_state_num = 0  # GET_TB3_DIRECTION

            elif self.turtlebot3_state_num == 2:  # TB3_RIGHT_TURN
                if abs(self.prev_tb3_pose - self.tb3_pose) >= self.escape_range:
                    self.turtlebot3_state_num = 0  # GET_TB3_DIRECTION
                else:
                    self.update_command_velocity(0.0, random.uniform(-1.0, 0))  # Right turn with angular velocity

            elif self.turtlebot3_state_num == 3:  # TB3_LEFT_TURN
                if abs(self.prev_tb3_pose - self.tb3_pose) >= self.escape_range:
                    self.turtlebot3_state_num = 0  # GET_TB3_DIRECTION
                else:
                    self.update_command_velocity(0.0, random.uniform(0, 1.0))  # Left turn with angular velocity

            rospy.sleep(0.001)

if __name__ == '__main__':
    try:
        turtlebot3_drive = Turtlebot3Drive()
        turtlebot3_drive.control_loop()
    except rospy.ROSInterruptException:
        pass
