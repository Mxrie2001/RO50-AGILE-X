#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import time
import sys

def move_forward(v_time):
	# Initialize the ROS node
	rospy.init_node('move_forward_node', anonymous=True)

	# Create a publisher for the robot's velocity commands
	cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

	# Create a Twist message to represent the velocity command
	twist = Twist()
	twist.linear.x = 0.1  # Set linear velocity (forward speed in m/s)
	twist.angular.z = 0  # Set angular velocity (turning speed in rad/s)
	start_time = time.time()

	# Publish the velocity command repeatedly to make the robot move forward
	rate = rospy.Rate(10)  # 10 Hz
	rospy.sleep(1)
	while(time.time() - start_time) < v_time:
		cmd_vel_pub.publish(twist)
		rate.sleep()
	print("Stop")
	twist.linear.x = 0.0
	cmd_vel_pub.publish(twist)	
	#rospy.sleep(1)
	print("Demi tour")
	twist.angular.z = 6.5
	cmd_vel_pub.publish(twist)
	rospy.sleep(2)
	
	twist.angular.z = 0.0
	cmd_vel_pub.publish(twist)
	while(time.time() - start_time) < v_time:
		cmd_vel_pub.publish(twist)
		rate.sleep()
	
if __name__ == '__main__':
	continuer = True
	try:
		while(continuer):
			move_forward(10)
	except rospy.ROSInterruptException:
		continuer = False
		sys.exit(0)
	except KeyboardInterrupt:
		continuer = False
		sys.exit(0)
