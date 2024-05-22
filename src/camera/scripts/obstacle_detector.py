#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Int32
from scipy.ndimage import median_filter
from message_filters import Subscriber, TimeSynchronizer, ApproximateTimeSynchronizer
import os

class ObstacleDetector:
    def __init__(self):
        self.rate = rospy.Rate(10)  # 10 Hz
        self.bridge = CvBridge()

        # Use message_filters to subscribe to color and depth image topics
        self.image_sub = Subscriber("/camera/color/image_raw", Image)
        self.depth_sub = Subscriber("/camera/depth/image_rect_raw", Image)

        # Use ApproximateTimeSynchronizer to sync the messages
        self.ts = ApproximateTimeSynchronizer([self.image_sub, self.depth_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.synced_callback)

        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.brake_pub = rospy.Publisher('/emergency_brake', Int32, queue_size=1)
        self.depth_image = None

        rospy.loginfo("Obstacle Detector Initialized")
        self.rate.sleep()

    def synced_callback(self, color_data, depth_data):
        try:
            color_image = self.bridge.imgmsg_to_cv2(color_data, "bgr8")
            depth_image_cv = self.bridge.imgmsg_to_cv2(depth_data, desired_encoding="passthrough")

            # Scale depth values if necessary
            if depth_data.encoding == "16UC1":  # Depth values are encoded as unsigned 16-bit integers
                depth_scale = 0.001  # Scaling factor for converting millimeters to meters
                depth_image_cv = depth_image_cv * depth_scale  # Convert depth values to meters
            elif depth_data.encoding == "32FC1":  # Depth values are encoded as 32-bit floating-point numbers
                pass  # No scaling needed for 32-bit float encoding

            self.depth_image = depth_image_cv
            self.process_image(color_image)

            rospy.loginfo("Color image shape: %s", color_image.shape)
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

    def process_image(self, color_image):
        if self.depth_image is not None:
            # Detect obstacles within 1 meter
            self.depth_image = self.depth_image[:, 104:744]
            self.depth_image = np.where(self.depth_image < 0.7, 10, self.depth_image)
            self.depth_image = median_filter(self.depth_image, size=3)
            self.depth_image[self.depth_image < 0.7] = 10

            save_filter_image = (self.depth_image * 25).astype(np.uint8)
            cv_mat = cv2.cvtColor(save_filter_image, cv2.COLOR_GRAY2BGR)

            # Save the image in the current working directory
            save_filename = "filter_image_depth.png"
            save_path = os.path.join(os.getcwd(), save_filename)

            # Attempt to save the image and log success or failure
            if cv2.imwrite(save_path, cv_mat):
                rospy.loginfo(f"Depth image saved successfully at: {save_path}")
            else:
                rospy.logerr(f"Failed to save depth image at: {save_path}")

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

        # Send msg for 1 second to ensure the robot receives the information
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

