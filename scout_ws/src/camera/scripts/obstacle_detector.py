#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Int32
from scipy.ndimage import median_filter

class ObstacleDetector:
    def __init__(self):
        self.rate = rospy.Rate(10)  # 10 Hz
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        self.depth_sub = rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.depth_callback)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.brake_pub = rospy.Publisher('/emergency_brake', Int32, queue_size=1)
        self.depth_image = None
        rospy.loginfo("Obstacle Detector Initialized")
        self.rate.sleep()

    def image_callback(self, data):
        try:
            color_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.process_image(color_image)
            rospy.loginfo(color_image.shape)
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

    def depth_callback(self, data):
        try:
            depth_image_cv = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
            cv2.imwrite("~/scout_ws/filter_image_depth.png", depth_image_cv)
            # Scale depth values if necessary
            if data.encoding == "16UC1":  # Depth values are encoded as unsigned 16-bit integers
                depth_scale = 0.001  # Scaling factor for converting millimeters to meters
                depth_image_cv = depth_image_cv * depth_scale  # Convert depth values to meters
            elif data.encoding == "32FC1":  # Depth values are encoded as 32-bit floating-point numbers
                pass  # No scaling needed for 32-bit float encoding
                    # Store the depth image
            self.depth_image = depth_image_cv
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

    def process_image(self, color_image):
        if self.depth_image is not None:
            # Detect obstacles within 1 meter
            self.depth_image = self.depth_image[:,104:744]
            self.depth_image = np.where(self.depth_image < 0.7, 10, self.depth_image)
            self.depth_image = median_filter(self.depth_image, size=3)
            self.depth_image[self.depth_image < 0.7] = 10
            save_filter_image = (self.depth_image * 25).astype(np.uint8)
            cv_mat = cv2.cvtColor(save_filter_image, cv2.COLOR_RGB2BGR)
            cv2.imwrite("~/scout_ws/filter_image_depth.png", cv_mat)
            count_images = np.sum(self.depth_image < 1)
            rospy.loginfo(self.depth_image.shape)
            rospy.loginfo(self.depth_image)
            rospy.loginfo(count_images)
            obstacle_mask = (self.depth_image < 1.0)  # Threshold for obstacle detection
            obstacle_detected = np.any(obstacle_mask)

            if obstacle_detected:
                rospy.loginfo("Obstacle detected! Stopping the robot.")
                self.stop_robot()

    def stop_robot(self):
        start_time = rospy.get_time()

	#sending msg for 1 second to be sure the robot received the information
        while not rospy.is_shutdown() and rospy.get_time() - start_time < 1:
            brake_trigger = 1
            self.brake_pub.publish(brake_trigger)
            rospy.loginfo("Emergency brake signal sent: %d", brake_trigger)

            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('obstacle_detector', anonymous=True)
    detector = ObstacleDetector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
    cv2.destroyAllWindows()
