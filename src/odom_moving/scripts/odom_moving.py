#!/usr/bin/env python3

import rospy
import math
import cv2
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, String, Float32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

#need to use odom_moving_turtlebot.launch with turtlebot for rviz

INITIATE_FORWARD = 1
KEEP_FORWARD = 2
INITIATE_TURN = 3
KEEP_TURN = 4
SLOWING_DOWN = 5
STOP_FOR_PEDESTRIAN = 6
EMERGENCY_STOP = 7

class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0
        self.integral = 0

    def calculate(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        output = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)
        self.prev_error = error
        return output

class StateMachine() :

    def __init__(self) -> None:
        self.state = INITIATE_FORWARD
        # Transition dictionary
        self.transitions = {
            INITIATE_FORWARD: [KEEP_FORWARD, EMERGENCY_STOP],
            KEEP_FORWARD: [KEEP_FORWARD, INITIATE_TURN, SLOWING_DOWN, STOP_FOR_PEDESTRIAN, EMERGENCY_STOP],
            INITIATE_TURN: [KEEP_TURN],
            KEEP_TURN: [KEEP_TURN, INITIATE_FORWARD, EMERGENCY_STOP],
            SLOWING_DOWN: [STOP_FOR_PEDESTRIAN, INITIATE_FORWARD, EMERGENCY_STOP],
            STOP_FOR_PEDESTRIAN: [INITIATE_FORWARD, EMERGENCY_STOP],
            EMERGENCY_STOP: []
        }

    def switch_state(self, new_state):
        # Check if the new state is accessible in the transitions
        if new_state in self.transitions[self.state]:
            self.state = new_state
        else:
            print(f"Cannot switch to state {new_state} from {self.state}")

class Odom:
    def __init__(self):

        self.rate = rospy.Rate(10)  # 10 Hz
        self.bridge = CvBridge()

        self.state_machine = StateMachine()
        self.error = 0
        self.odom_msg_old_ = None
        self.twist_msg_ = Twist()
        self.frame_without_human = 0
        self.lidar_frame_without_human = 0
        self.distance = 8
        self.distance_parcouru = 0
        self.speed = 0.3
        self.max_speed = 1

        # Publishers
        self.moving_pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        # Subscribers
        rospy.Subscriber('/odom', Odometry, self.odomCallback)
        rospy.Subscriber('/emergency_brake', Int32, self.emergencyBrakeCallback) #trigger subscriber
        # rospy.Subscriber('/emergency_brake_reason', String, self.emergencyBrakeReasonCallback) #reason subscriber
        # rospy.Subscriber("/human_distances", Float32MultiArray, self.humanCallback)
        # rospy.Subscriber("/lidar_detection", Float32MultiArray, self.lidarCallback)
        # rospy.Subscriber("/lidar_pedestrian_stop", Int32, self.lidarPedeCallback)
        rospy.Subscriber("/human_distances", Float32MultiArray, self.distancesCallback)
        
        #rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)

        self.pid_controller = PIDController(Kp=0.6, Ki=0.2, Kd=0)  # Ajustez les coefficients PID selon vos besoins

        self.rate.sleep()
    
    def odomCallback(self, odom_msg):  
        if self.state_machine.state == INITIATE_FORWARD:
            self.odom_msg_old_ = odom_msg
            self.twist_msg_.linear.x = self.speed
            self.distance = self.distance - self.distance_parcouru
            self.moving_pub_.publish(self.twist_msg_)
            self.state_machine.switch_state(KEEP_FORWARD)
        elif self.state_machine.state == KEEP_FORWARD:  # forward
            dist_ = odom_msg.twist.twist.linear.x * (odom_msg.header.stamp.to_sec() - self.odom_msg_old_.header.stamp.to_sec())
            self.moving_pub_.publish(self.twist_msg_)
            self.distance_parcouru = dist_
            if dist_ >= self.distance:
                self.distance = 8
                self.distance_parcouru = 0
                self.twist_msg_.linear.x = 0.0
                self.moving_pub_.publish(self.twist_msg_)
                self.state_machine.switch_state(INITIATE_TURN)
        elif self.state_machine.state == INITIATE_TURN:
            self.odom_msg_old_ = odom_msg
            self.twist_msg_.angular.z = 0.2
            self.moving_pub_.publish(self.twist_msg_)
            self.state_machine.switch_state(KEEP_TURN)
        elif self.state_machine.state == KEEP_TURN:  # turn
            current_time = odom_msg.header.stamp.to_sec()
            previous_time = self.odom_msg_old_.header.stamp.to_sec()
            dt = current_time - previous_time
            angle_ = odom_msg.twist.twist.angular.z * dt
            if angle_ >= math.pi:
                self.twist_msg_.angular.z = 0
                self.moving_pub_.publish(self.twist_msg_)
                self.state_machine.switch_state(INITIATE_FORWARD)
            self.moving_pub_.publish(self.twist_msg_)
            # self.error = math.pi - angle_
            # if abs(self.error) < 0.03:
            #     rospy.loginfo(self.error)
            #     rospy.loginfo(angle_)
            #     self.twist_msg_.angular.z = 0
            #     self.moving_pub_.publish(self.twist_msg_)
            #     self.state_machine.switch_state(INITIATE_FORWARD)
            #     return
            # self.moving_pub_.publish(self.twist_msg_)
            # pid_output = self.pid_controller.calculate(self.error, dt)
            # self.twist_msg_.angular.z = pid_output
        # elif self.state_machine.state == SLOWING_DOWN:
        #     self.twist_msg_.linear.x = 0.2
        #     self.moving_pub_.publish(self.twist_msg_)
        elif self.state_machine.state == STOP_FOR_PEDESTRIAN:
            start_time = rospy.get_time()
            while not rospy.is_shutdown() and rospy.get_time() - start_time < 4:
                self.twist_msg_.linear.x = 0.0
                self.moving_pub_.publish(self.twist_msg_)
            self.odom_msg_old_ = odom_msg
            self.state_machine.switch_state(INITIATE_FORWARD)
        elif self.state_machine.state == EMERGENCY_STOP:
            self.twist_msg_.linear.x = 0.0
            self.twist_msg_.angular.z = 0.0
            self.moving_pub_.publish(self.twist_msg_)

    #trigger callback
    def emergencyBrakeCallback(self, msg):
        #message received from /emergency_brake topic
        if msg.data != 0:
            rospy.loginfo("Emergency brake signal received. Stopping the robot.")
            self.twist_msg_.linear.x = 0.0
            self.twist_msg_.angular.z = 0.0
            self.moving_pub_.publish(self.twist_msg_)
            self.state_machine.switch_state(EMERGENCY_STOP)
    
    def distancesCallback(self, msg):
        if(len(msg.data) > 0):
            self.frame_without_human = 0
            mini = min (msg.data)
            speedf = min(max(0.4 * (mini - 2.8), 0), 40)
            speeda = min(max(0.4 *(mini - 4), 0, self.speed), 40)
            self.speed = min(speedf, speeda)*0.01*self.max_speed
            if(self.speed < 0.05):
                self.state_machine.switch_state(STOP_FOR_PEDESTRIAN)

        else:
            self.speed = 0.4

    # #reason callback
    # def emergencyBrakeReasonCallback(self, msg):
    #     emergency_brake_reason_ = msg.data
    #     rospy.loginfo("Braking reason: %s", emergency_brake_reason_)
    
    # def humanCallback(self, msg):       
    #     if(len(msg.data) > 0):
    #         self.frame_without_human = 0
    #         mini = min(msg.data)
    #         if mini <= 2:
    #             self.state_machine.switch_state(STOP_FOR_PEDESTRIAN)
    #     else:
    #         self.frame_without_human += 1
    #         if self.frame_without_human > 4:
    #             self.state_machine.switch_state(INITIATE_FORWARD)
    
    # def lidarCallback(self, msg):
    #     if(len(msg.data) > 0):
    #         self.frame_without_human = 0
    #         mini = min(msg.data)
    #         if mini <= 4 and mini > 2:
    #             self.state_machine.switch_state(SLOWING_DOWN)
    #         elif mini <= 2:
    #             self.state_machine.switch_state(STOP_FOR_PEDESTRIAN)
    #     else:
    #         self.frame_without_human += 1
    #         if(self.frame_without_human > 4):
    #             self.state_machine.switch_state(INITIATE_FORWARD)
    
    # def lidarPedeCallback(self, msg):
    #     if msg.data != 0:
    #         self.lidar_frame_without_human = 0
    #         self.state_machine.switch_state(STOP_FOR_PEDESTRIAN)
    #     else:
    #         self.lidar_frame_without_human += 1
    #         if self.lidar_frame_without_human > 4 :
    #             self.state_machine.switch_state(INITIATE_FORWARD)

    # def image_callback(self, color_data):
    #     if(self.step_ == 0):
    #         try:
    #             color_image = self.bridge.imgmsg_to_cv2(color_data, "bgr8")
    #             self.process_image(color_image)
    #             rospy.loginfo("Color image shape: %s", color_image.shape)
    #         except CvBridgeError as e:
    #             rospy.logerr("CvBridge Error: {0}".format(e))

    # def process_image(self, color_image):
    #     self.detect_lines(color_image)

    # def detect_lines(self, color_image):
    #     # Convert the image to grayscale
    #     gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

    #     # Extract the region of interest (ROI)
    #     scaled_image = gray_image[390:480, 0:640]

    #     # Threshold the grayscale image to get a binary mask for yellow regions
    #     _, mask = cv2.threshold(scaled_image, 180, 255, cv2.THRESH_BINARY)

    #     # Apply another mask to remove isolated pixels
    #     kernel = np.ones((3, 3), np.uint8)
    #     opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    #     # Find edges using Canny edge detector
    #     edges = cv2.Canny(opening, 50, 150)

    #     # Use Hough Line Transform to detect lines
    #     lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=50, minLineLength=50, maxLineGap=10)

    #     # Display the mask with edges for debugging
    #     cv2.imshow("Yellow Line Detection", edges)
    #     cv2.waitKey(1)

    #     if lines is not None:
    #         rospy.loginfo("Lines detected: %d", len(lines))

    #         # Separate lines into left and right based on their slope
    #         left_lines = []
    #         right_lines = []
    #         for line in lines:
    #             x1, y1, x2, y2 = line[0]
    #             if x1 == x2:
    #                 continue  # Skip vertical lines
    #             slope = (y2 - y1) / (x2 - x1)
    #             if slope < 0:
    #                 left_lines.append(line)
    #             else:
    #                 right_lines.append(line)

    #         if len(left_lines) > 0 and len(right_lines) > 0:
    #             # Calculate the average position of the left and right lines
    #             left_x1 = np.mean([line[0][0] for line in left_lines])
    #             left_x2 = np.mean([line[0][2] for line in left_lines])
    #             left_y1 = np.mean([line[0][1] for line in left_lines])
    #             left_y2 = np.mean([line[0][3] for line in left_lines])

    #             right_x1 = np.mean([line[0][0] for line in right_lines])
    #             right_x2 = np.mean([line[0][2] for line in right_lines])
    #             right_y1 = np.mean([line[0][1] for line in right_lines])
    #             right_y2 = np.mean([line[0][3] for line in right_lines])

    #             # Draw the average lines on the image
    #             cv2.line(color_image, (int(left_x1), int(left_y1)), (int(left_x2), int(left_y2)), (0, 0, 255), 2)
    #             cv2.line(color_image, (int(right_x1), int(right_y1)), (int(right_x2), int(right_y2)), (0, 0, 255), 2)

    #             # Calculate the midpoint between the average left and right lines
    #             mid_x = (left_x1 + right_x2) / 2
    #             mid_y = (left_y1 + right_y2) / 2

    #             # Mark the midpoint on the image
    #             cv2.circle(color_image, (int(mid_x), int(mid_y)), 5, (255, 0, 0), -1)

    #             # Control the robot to stay centered between the lines
    #             self.control_robot(mid_x, color_image.shape[1] / 2)
    #         elif len(left_lines) == 0 and len(right_lines) > 0:
    #             # Turn left to find the left line
    #             rospy.loginfo("Turn left to find the left line")
    #             # Implement the action to turn left
    #             self.turn_left()
    #         elif len(left_lines) > 0 and len(right_lines) == 0:
    #             # Turn right to find the right line
    #             rospy.loginfo("Turn right to find the right line")
    #             # Implement the action to turn right
    #             self.turn_right()
    #     else:
    #         rospy.loginfo("No lines detected.")


    # def control_robot(self, mid_x, image_center_x):
    #     error = mid_x - image_center_x
    #     rospy.loginfo(error)

    #     if(error < 0.4 and error > -0.4):
    #         rospy.loginfo("error mini")
    #         self.step_ = 1
    #         return

    #     # Proportional control for centering the robot
    #     k_p = 0.01  # Proportional gain
    #     twist = Twist()
    #     twist.angular.z = -k_p * error / 3  # Adjust angular speed based on the error

    #     self.moving_pub_.publish(twist)
    #     rospy.loginfo("Control command published: linear.x = %.2f, angular.z = %.2f", twist.linear.x, twist.angular.z)
    
    # def turn_right(self):
    #     twist = Twist()
    #     twist.angular.z = -0.05
    #     self.moving_pub_.publish(twist)
    
    # def turn_left(self):
    #     twist = Twist()
    #     twist.angular.z = 0.05
    #     self.moving_pub_.publish(twist)

if __name__ == '__main__':
    rospy.init_node('odom_moving')
    odom = Odom()
    rospy.spin()