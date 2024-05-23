#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Int32

class LineDetector:
    def __init__(self):
        self.rate = rospy.Rate(10)  # 10 Hz
        self.bridge = CvBridge()

        # Subscribe to color image topic
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)

        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        rospy.loginfo("Line Detector Initialized")
        self.rate.sleep()

    def image_callback(self, color_data):
        try:
            color_image = self.bridge.imgmsg_to_cv2(color_data, "bgr8")
            self.process_image(color_image)
            rospy.loginfo("Color image shape: %s", color_image.shape)
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

    def process_image(self, color_image):
        self.detect_lines(color_image)

    def detect_lines(self, color_image):
        # Convert the image to grayscale
        gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

        # Extract the region of interest (ROI)
        scaled_image = gray_image[350:480, 0:640]

        # Threshold the grayscale image to get a binary mask for yellow regions
        _, mask = cv2.threshold(scaled_image, 180, 255, cv2.THRESH_BINARY)

        # Apply another mask to remove isolated pixels
        kernel = np.ones((3, 3), np.uint8)
        opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        # Find edges using Canny edge detector
        edges = cv2.Canny(opening, 50, 150)

        # Use Hough Line Transform to detect lines
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=50, minLineLength=50, maxLineGap=10)

        # Display the mask with edges for debugging
        cv2.imshow("Yellow Line Detection", edges)
        cv2.waitKey(1)

        if lines is not None:
            rospy.loginfo("Lines detected: %d", len(lines))

            # Separate lines into left and right based on their slope
            left_lines = []
            right_lines = []
            for line in lines:
                x1, y1, x2, y2 = line[0]
                if x1 == x2:
                    continue  # Skip vertical lines
                slope = (y2 - y1) / (x2 - x1)
                if slope < 0:
                    left_lines.append(line)
                else:
                    right_lines.append(line)

            if len(left_lines) > 0 and len(right_lines) > 0:
                # Calculate the average position of the left and right lines
                left_x1 = np.mean([line[0][0] for line in left_lines])
                left_x2 = np.mean([line[0][2] for line in left_lines])
                left_y1 = np.mean([line[0][1] for line in left_lines])
                left_y2 = np.mean([line[0][3] for line in left_lines])

                right_x1 = np.mean([line[0][0] for line in right_lines])
                right_x2 = np.mean([line[0][2] for line in right_lines])
                right_y1 = np.mean([line[0][1] for line in right_lines])
                right_y2 = np.mean([line[0][3] for line in right_lines])

                # Draw the average lines on the image
                cv2.line(color_image, (int(left_x1), int(left_y1)), (int(left_x2), int(left_y2)), (0, 0, 255), 2)
                cv2.line(color_image, (int(right_x1), int(right_y1)), (int(right_x2), int(right_y2)), (0, 0, 255), 2)

                # Calculate the midpoint between the average left and right lines
                mid_x = (left_x1 + right_x2) / 2
                mid_y = (left_y1 + right_y2) / 2

                # Mark the midpoint on the image
                cv2.circle(color_image, (int(mid_x), int(mid_y)), 5, (255, 0, 0), -1)

                # Control the robot to stay centered between the lines
                self.control_robot(mid_x, color_image.shape[1] / 2)
            elif len(left_lines) == 0 and len(right_lines) > 0:
                # Turn left to find the left line
                rospy.loginfo("Turn left to find the left line")
                # Implement the action to turn left
                self.turn_left()
            elif len(left_lines) > 0 and len(right_lines) == 0:
                # Turn right to find the right line
                rospy.loginfo("Turn right to find the right line")
                # Implement the action to turn right
                self.turn_right()
        else:
            rospy.loginfo("No lines detected.")


    def control_robot(self, mid_x, image_center_x):
        error = mid_x - image_center_x

        # Proportional control for centering the robot
        k_p = 0.01  # Proportional gain
        twist = Twist()
        #twist.linear.x = 0.2  # Constant forward speed
        twist.angular.z = -k_p * error  # Adjust angular speed based on the error

        rospy.loginfo(error)

        self.cmd_pub.publish(twist)
        rospy.loginfo("Control command published: linear.x = %.2f, angular.z = %.2f", twist.linear.x, twist.angular.z)
    
    def turn_right(self):
        twist = Twist()
        twist.angular.z = -0.1
        self.cmd_pub.publish(twist)
    
    def turn_left(self):
        twist = Twist()
        twist.angular.z = 0.1
        self.cmd_pub.publish(twist)

if __name__ == '__main__':
    rospy.init_node('line_detector', anonymous=True)
    detector = LineDetector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
    cv2.destroyAllWindows()