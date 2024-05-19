#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

def move_robot():
    rospy.init_node('move_robot_node', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)  # 10Hz
    while not rospy.is_shutdown():
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.2  # adjust speed as needed
        cmd_vel.angular.z = 0.0
        pub.publish(cmd_vel)
        rate.sleep()

if __name__ == '__main__':
    try:
        move_robot()
    except rospy.ROSInterruptException:
        pass

