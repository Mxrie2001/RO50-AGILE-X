#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Int32
from scipy.ndimage import median_filter
from message_filters import Subscriber, ApproximateTimeSynchronizer
from visualization_msgs.msg import Marker, MarkerArray
import os
from imutils.object_detection import non_max_suppression

class ObstacleDetector:
    def __init__(self):
        self.rate = rospy.Rate(10)  # 10 Hz
        self.bridge = CvBridge()

        # Use message_filters to subscribe to color and depth image topics
        self.image_sub = Subscriber("/camera/color/image_raw", Image)
        self.depth_sub = Subscriber("/camera/depth/image_rect_raw", Image)
        self.lidar_sub = Subscriber("/scan", LaserScan)

        # Use ApproximateTimeSynchronizer to sync the messages
        self.ts = ApproximateTimeSynchronizer([self.image_sub, self.depth_sub, self.lidar_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.synced_callback)

        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.brake_pub = rospy.Publisher('/emergency_brake', Int32, queue_size=1)
        self.marker_pub = rospy.Publisher('pedestrian_markers', MarkerArray, queue_size=10)
        
        self.depth_image = None

        rospy.loginfo("Obstacle and Pedestrian Detector Initialized")
        self.rate.sleep()

    def synced_callback(self, color_data, depth_data, lidar_data):
        try:
            color_image = self.bridge.imgmsg_to_cv2(color_data, "bgr8")
            depth_image_cv = self.bridge.imgmsg_to_cv2(depth_data, desired_encoding="passthrough")

            # Ensure the depth values are correctly scaled
            if depth_data.encoding == "16UC1":  # Depth values are encoded as unsigned 16-bit integers
                depth_scale = 0.001  # Scaling factor for converting millimeters to meters
                depth_image_cv = depth_image_cv * depth_scale  # Convert depth values to meters
            elif depth_data.encoding == "32FC1":  # Depth values are encoded as 32-bit floating-point numbers
                pass  # No scaling needed for 32-bit float encoding

            self.depth_image = depth_image_cv
            self.process_image()
            self.process_lidar(lidar_data)
            #self.process_image_rgb(color_image)
            self.detect_humans(color_image)

            rospy.loginfo("Color image shape: %s", color_image.shape)
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

    def process_image(self):
        if self.depth_image is not None:
            # Detect obstacles within 2 meters
            self.depth_image = self.depth_image[:, 104:744]
            self.depth_image = np.where(self.depth_image < 2.0, 10, self.depth_image)
            self.depth_image = median_filter(self.depth_image, size=3)
            self.depth_image[self.depth_image < 2.0] = 10

            save_filter_image = (self.depth_image * 25).astype(np.uint8)
            cv_mat = cv2.cvtColor(save_filter_image, cv2.COLOR_GRAY2BGR)

            # # Save the image in the current working directory
            # save_filename = "filter_image_depth.png"
            # save_path = os.path.join(os.getcwd(), save_filename)

            # # Attempt to save the image and log success or failure
            # if cv2.imwrite(save_path, cv_mat):
            #     rospy.loginfo(f"Depth image saved successfully at: {save_path}")
            # else:
            #     rospy.logerr(f"Failed to save depth image at: {save_path}")

            count_images = np.sum(self.depth_image < 2)
            rospy.loginfo(self.depth_image.shape)
            rospy.loginfo(self.depth_image)
            rospy.loginfo(count_images)

            obstacle_mask = (self.depth_image < 2.0)  # Threshold for obstacle detection
            obstacle_detected = np.any(obstacle_mask)

            if obstacle_detected:
                rospy.loginfo("Obstacle detected! Stopping the robot.")
                self.stop_robot()
    
    def process_image_rgb(self, color_image):

        # hog = cv2.HOGDescriptor()
        # hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

        # (regions, weights) = hog.detectMultiScale(color_image, winStride=(4,4), padding=(8,8), scale = 1.05)
        
        # rects = np.array([[x, y, x + w, y + h] for (x, y, w, h) in regions])
        # pick = non_max_suppression(rects, probs=None, overlapThresh = 0.65)

        # for(x, y, x2, y2) in pick:
        #     cv2.rectangle(color_image, (x, y), (x2, y2), (0, 0, 255), 2)

        body=cv2.CascadeClassifier('/home/hugoc/RO50_project_ws/src/camera/scripts/haarcascade_fullbody.xml')
        detection=body.detectMultiScale(color_image,scaleFactor=1.05,minSize=(50,50))
        for (x,y,w,h) in detection:
            cv2.rectangle(color_image,(x,y),(x+w,y+h),(0,0,255),2)
        
        # save_filename = "rgb_image.png"
        # save_path = os.path.join(os.getcwd(), save_filename)

        # # Attempt to save the image and log success or failure
        # if cv2.imwrite(save_path, color_image):
        #     rospy.loginfo(f"Depth image saved successfully at: {save_path}")
        # else:
        #     rospy.logerr(f"Failed to save depth image at: {save_path}")

    def process_lidar(self, data):
        ranges = np.array(data.ranges)
        angle_increment = data.angle_increment
        total_angles = len(ranges)
        total_angle_range = 360.0  # Assuming the LiDAR covers a full 360 degrees

        # Calculate the number of indices corresponding to 130 degrees
        angle_range = 120.0
        indices_range = int((angle_range / total_angle_range) * total_angles)

        # Center the 130 degree range in front of the robot
        center_index = total_angles // 2
        start_index = center_index - indices_range // 2
        end_index = center_index + indices_range // 2

        # Filter the ranges to include only the desired 130 degree field of view
        filtered_ranges = np.concatenate((ranges[start_index:], ranges[:end_index])) if start_index < 0 else ranges[start_index:end_index]

        distance_threshold = 0.2  # Distance threshold for clustering in meters
        target_distance = 2.0     # Target distance for pedestrian detection
        clusters = self.euclidean_clustering(filtered_ranges, angle_increment, distance_threshold)
        
        min_size = 5   # Minimum number of points in a cluster to be considered a pedestrian
        max_size = 50  # Maximum number of points in a cluster to be considered a pedestrian
        pedestrian_clusters = self.detect_pedestrians(clusters, min_size, max_size, target_distance, distance_threshold)
        
        if pedestrian_clusters:
            rospy.loginfo("Pedestrians detected: %d", len(pedestrian_clusters))
            self.publish_markers(pedestrian_clusters)
            self.stop_robot()

    def euclidean_clustering(self, ranges, angle_increment, distance_threshold):
        clusters = []
        cluster = []
        for i in range(len(ranges)):
            if np.isinf(ranges[i]):
                continue
            angle = i * angle_increment
            if not cluster:
                cluster.append((angle, ranges[i]))
            elif abs(ranges[i] - cluster[-1][1]) < distance_threshold:
                cluster.append((angle, ranges[i]))
            else:
                clusters.append(cluster)
                cluster = [(angle, ranges[i])]
        if cluster:
            clusters.append(cluster)
        return clusters

    def detect_pedestrians(self, clusters, min_size, max_size, target_distance, distance_threshold):
        pedestrian_clusters = []
        for cluster in clusters:
            if min_size < len(cluster) < max_size:
                mean_distance = np.mean([point[1] for point in cluster])
                if abs(mean_distance - target_distance) <= distance_threshold:
                    pedestrian_clusters.append((cluster, mean_distance))
        return pedestrian_clusters
    
    def detect_humans(self, image):
        # Load YOLO
        net = cv2.dnn.readNet("/home/hugoc/yolov3.weights", "/home/hugoc/RO50_project_ws/src/camera/scripts/yolov3.cfg")
        with open("/home/hugoc/RO50_project_ws/src/camera/scripts/coco.names", "r") as f:
            classes = [line.strip() for line in f.readlines()]

        layer_names = net.getLayerNames()
        output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]

        height, width, channels = image.shape

        # Detecting objects
        blob = cv2.dnn.blobFromImage(image, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
        net.setInput(blob)
        outs = net.forward(output_layers)

        # Showing information on the screen
        class_ids = []
        confidences = []
        boxes = []
        for out in outs:
            for detection in out:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                if confidence > 0.5 and class_id == 0:  # Only detecting humans (class_id == 0 for person)
                    center_x = int(detection[0] * width)
                    center_y = int(detection[1] * height)
                    w = int(detection[2] * width)
                    h = int(detection[3] * height)

                    # Rectangle coordinates
                    x = int(center_x - w / 2)
                    y = int(center_y - h / 2)

                    boxes.append([x, y, w, h])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)

        indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)

        for i in range(len(boxes)):
            if i in indexes:
                x, y, w, h = boxes[i]
                label = str(classes[class_ids[i]])
                confidence = confidences[i]
                color = (0, 255, 0)  # Green color for bounding box
                cv2.rectangle(image, (x, y), (x + w, y + h), color, 2)
                cv2.putText(image, label + " " + str(round(confidence, 2)), (x, y + 30), cv2.FONT_HERSHEY_PLAIN, 3, color, 3)
        
        # Save the RGB image with detections
        save_filename = "rgb_image_yolo.png"
        save_path = os.path.join(os.getcwd(), save_filename)
        if cv2.imwrite(save_path, image):
            rospy.loginfo(f"RGB image saved successfully at: {save_path}")
        else:
            rospy.logerr(f"Failed to save RGB image at: {save_path}")

    def publish_markers(self, pedestrian_clusters):
        marker_array = MarkerArray()
        for idx, (cluster, mean_distance) in enumerate(pedestrian_clusters):
            marker = Marker()
            marker.header.frame_id = "laser_frame"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "pedestrians"
            marker.id = idx
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0

            # Position du centre du cluster
            x = np.mean([data[1] * np.cos(data[0]) for data in cluster])
            y = np.mean([data[1] * np.sin(data[0]) for data in cluster])
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0

            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)

    def stop_robot(self):
        start_time = rospy.get_time()

        # Send msg for 1 second to ensure the robot receives the information
        while not rospy.is_shutdown() and rospy.get_time() - start_time < 1:
            brake_trigger = 1
            self.brake_pub.publish(brake_trigger)
            rospy.loginfo("Emergency brake signal sent: %d", brake_trigger)

            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('obstacle_and_pedestrian_detector', anonymous=True)
    detector = ObstacleDetector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
    cv2.destroyAllWindows()