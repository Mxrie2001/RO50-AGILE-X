#!/usr/bin/env python3

import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, String

moving_pub_ = None
odom_msg_old_ = Odometry()
twist_msg_ = Twist()
step_ = 0
distance_, dist_, angle_ = 0.0, 0.0, 0.0
emergency_brake_reason_ = "Unknown reason"

#need to use odom_moving_turtlebot.launch with turtlebot for rviz

def odomCallback(odom_msg):
    global step_, odom_msg_old_, twist_msg_, dist_, angle_
    
    if step_ == 0:
        odom_msg_old_ = odom_msg
        twist_msg_.linear.x = 0.2
        moving_pub_.publish(twist_msg_)
        step_ = 1
    elif step_ == 1:  # forward
        dist_ = odom_msg.twist.twist.linear.x * (odom_msg.header.stamp.to_sec() - odom_msg_old_.header.stamp.to_sec())
        moving_pub_.publish(twist_msg_)
        if dist_ >= 2:
            twist_msg_.linear.x = 0.0
            moving_pub_.publish(twist_msg_)
            step_ = 2
    elif step_ == 2:
        odom_msg_old_ = odom_msg
        twist_msg_.angular.z = 0.2
        moving_pub_.publish(twist_msg_)
        step_ = 3
    elif step_ == 3:  # turn
        angle_ = odom_msg.twist.twist.angular.z * (odom_msg.header.stamp.to_sec() - odom_msg_old_.header.stamp.to_sec())
        moving_pub_.publish(twist_msg_)
        if angle_ >= math.pi:
            twist_msg_.angular.z = 0.0  
            moving_pub_.publish(twist_msg_)
            step_ = 0

#trigger callback
def emergencyBrakeCallback(msg):
    global twist_msg_
    #message received from /emergency_brake topic
    if msg.data != 0:
        rospy.loginfo("Emergency brake signal received. Stopping the robot.")
        twist_msg_.linear.x = 0.0
        twist_msg_.angular.z = 0.0
        moving_pub_.publish(twist_msg_)

#reason callback
def emergencyBrakeReasonCallback(msg):
    global emergency_brake_reason_
    emergency_brake_reason_ = msg.data
    rospy.loginfo("Braking reason: %s", emergency_brake_reason_)

def main():
    global moving_pub_, distance_
    
    rospy.init_node('odom_moving')
    
    # Publishers
    moving_pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    
    # Subscribers
    rospy.Subscriber('/odom', Odometry, odomCallback)
    rospy.Subscriber('/emergency_brake', Int32, emergencyBrakeCallback) #trigger subscriber
    rospy.Subscriber('/emergency_brake_reason', String, emergencyBrakeReasonCallback) #reason subscriber
   

    # Parameters
    rospy.get_param('~distance', distance_)
    
    rospy.spin()

if __name__ == '__main__':
    main()
