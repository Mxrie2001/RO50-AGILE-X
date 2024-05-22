#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32, String

def emergency_stop():
    rospy.init_node('emergency_stop')
    rate = rospy.Rate(10)  # 10 Hz

    #trigger publisher
    brake_pub = rospy.Publisher('/emergency_brake', Int32, queue_size=1)
    #reason publisher
    brake_reason_pub = rospy.Publisher('/emergency_brake_reason', String, queue_size=1)

    #waiting for publishers to be ready
    rate.sleep()

    start_time = rospy.get_time()

    #sending msg for 1 second to be sure the robot received the information
    while not rospy.is_shutdown() and rospy.get_time() - start_time < 1:
        brake_trigger = 1
        brake_pub.publish(brake_trigger)
        rospy.loginfo("Emergency brake signal sent: %d", brake_trigger)

        brake_reason = "Internal temperature is too high!"
        brake_reason_pub.publish(brake_reason)
        rospy.loginfo("Braking reason: %s", brake_reason)

        rate.sleep()

if __name__ == '__main__':
    emergency_stop()