#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Int32, Float32MultiArray
from message_filters import Subscriber, ApproximateTimeSynchronizer
from visualization_msgs.msg import Marker, MarkerArray
import os
from imutils.object_detection import non_max_suppression
import threading
from sklearn.cluster import DBSCAN
import tf
from tf import TransformListener

class ObstacleDetector:
    def __init__(self):
        self.rate = rospy.Rate(10)  # 10 Hz
        self.bridge = CvBridge()

        self.boxes = []
        self.depth_image = None
        self.processing_image = False
        self.distances = []
        # Initialize variables for robot's pose (x, y, theta)
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0

        # Load YOLO
        self.net = cv2.dnn.readNet("/home/hugoc/yolov3-tiny.weights", "/home/hugoc/RO50_project_ws/src/camera/scripts/yolov3-tiny.cfg")
        with open("/home/hugoc/RO50_project_ws/src/camera/scripts/coco.names", "r") as f:
            self.classes = [line.strip() for line in f.readlines()]

        layer_names = self.net.getLayerNames()
        self.output_layers = [layer_names[i[0] - 1] for i in self.net.getUnconnectedOutLayers()]

        self.processing_thread = None

        self.tf_listener = TransformListener()
        self.lidar_pub = rospy.Publisher('/human_distances', Float32MultiArray, queue_size=1)

        # Use message_filters to subscribe to color and depth image topics
        self.image_sub = Subscriber("/camera/color/image_raw", Image)
        self.depth_sub = Subscriber("/camera/depth/image_rect_raw", Image)
        self.lidar_sub = Subscriber("/scan", LaserScan)

        # Use ApproximateTimeSynchronizer to sync the messages
        self.ts = ApproximateTimeSynchronizer([self.image_sub, self.depth_sub, self.lidar_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.synced_callback)

        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.brake_pub = rospy.Publisher('/emergency_brake', Int32, queue_size=1)
        # self.human_pub = rospy.Publisher('/human_distances', Float32MultiArray, queue_size=1)
        # self.lidar_pub = rospy.Publisher('/lidar_detection', Float32MultiArray, queue_size=1)
        # self.lidar_pub_2 = rospy.Publisher('/lidar_pedestrian_stop', Int32, queue_size=1)

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

            # Process LIDAR data immediately
            self.process_lidar(lidar_data)

            # Start image processing in a separate thread if not already processing
            if not self.processing_image:
                self.processing_image = True
                self.processing_thread = threading.Thread(target=self.detect_humans, args=(color_image,))
                self.processing_thread.start()

        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

    def save_image(self, image, image_path):
        # Save the RGB image with detections
        save_filename = image_path
        save_path = os.path.join(os.getcwd(), save_filename)
        cv2.imwrite(save_path, image)

    def process_image_depth(self):
        if self.depth_image is not None:
            # Detect obstacles within 2 meters
            self.depth_image = self.depth_image[:, 104:744]
            self.depth_image = np.where(self.depth_image < 0.5, 10, self.depth_image)
            depth_image_uint8 = (self.depth_image * 255).astype(np.uint8)
            depth_image_blurred = cv2.medianBlur(depth_image_uint8, 3)
            self.depth_image = depth_image_blurred.astype(np.float32)/255.0
            self.depth_image[self.depth_image < 0.5] = 10

    def process_lidar(self, data):
        ranges = np.array(data.ranges)
        total_angles = len(ranges)
        total_angle_range = 360.0  # Assuming the LiDAR covers a full 360 degrees
        target_distance = 3
        threshold_point_cluster = 0.5

        # Calculate the number of indices corresponding to 150 degrees
        angle_range = 140.0
        indices_range = int((angle_range / total_angle_range) * total_angles)

        # Center the 150 degree range in front of the robot
        center_index = total_angles // 2
        start_index = center_index - indices_range // 2
        end_index = center_index + indices_range // 2

        # Filter the ranges to include only the desired 150 degree field of view
        if start_index < 0:
            filtered_ranges = np.concatenate((ranges[start_index:], ranges[:end_index]))
        else:
            filtered_ranges = ranges[start_index:end_index]
        filtered_ranges[filtered_ranges <= 0.1] = 100

        # Check for any object within 1 meter
        if np.any(filtered_ranges <= 1):
            rospy.loginfo("Obstacle detected within 1 meter! Stopping the robot.")
            self.stop_robot()
            return  # Exit the function early to immediately stop the robot

        len_range = len(filtered_ranges)
        current_angle = 0
        start_detecting_cluster = False
        last_point = None
        clusters = []
        cluster = [0,0,0]
        for filtered_point in filtered_ranges :
            if filtered_point <= target_distance :

                if start_detecting_cluster == False : 
                    clusters.append(cluster)
                    cluster = [0,0,0]

                if start_detecting_cluster and abs(last_point - filtered_point) >= threshold_point_cluster : 
                    start_detecting_cluster = False

                start_detecting_cluster = True

                cluster[0] += current_angle
                cluster[1] += filtered_point
                cluster[2] += 1
            else : 
                start_detecting_cluster = False
            
            current_angle += angle_range/ len_range
            last_point = filtered_point

        if start_detecting_cluster and cluster[2] > 0:
            clusters.append(cluster)
        
        filtered_clusters = []
        max_points = 15
        min_points = 2
        for cluster in clusters:
            if(cluster[2] > max_points or cluster[2] < min_points) or cluster[2] == 0:
                continue
            mean_distance = cluster[1]/cluster[2]
            mean_angle = cluster[0]/cluster[2]
            proj_x = abs(np.sin((np.radians(mean_angle)) * mean_distance))
            proj_y = abs(np.cos((np.radians(mean_angle)) * mean_distance))
            if proj_y > 0.75:
                continue
            filtered_cluster = [proj_x, proj_y, cluster[2]]
            filtered_clusters.append(filtered_cluster)

        cluster_distances = []
        for filtered_cluster in filtered_clusters:
            cluster_distances.append(filtered_cluster[0])

        array_msg = Float32MultiArray()
        array_msg.data = cluster_distances if cluster_distances else []
        self.lidar_pub.publish(array_msg)
        rospy.loginfo(filtered_clusters)

    # def process_lidar(self, data):
    #     ranges = np.array(data.ranges)
    #     angle_increment = data.angle_increment
    #     total_angles = len(ranges)
    #     total_angle_range = 360.0  # Assuming the LiDAR covers a full 360 degrees

    #     # Calculate the number of indices corresponding to 150 degrees
    #     angle_range = 150.0
    #     indices_range = int((angle_range / total_angle_range) * total_angles)

    #     # Center the 150 degree range in front of the robot
    #     center_index = total_angles // 2
    #     start_index = center_index - indices_range // 2
    #     end_index = center_index + indices_range // 2

    #     # Filter the ranges to include only the desired 150 degree field of view
    #     if start_index < 0:
    #         filtered_ranges = np.concatenate((ranges[start_index:], ranges[:end_index]))
    #     else:
    #         filtered_ranges = ranges[start_index:end_index]
    #     filtered_ranges[filtered_ranges <= 0] = 100

    #     distance_threshold = 0.2  # Distance threshold for clustering in meters
    #     target_distance = 5.0  # Target distance for pedestrian detection

    #     # Check for any object within 1 meter
    #     if np.any(filtered_ranges <= 1):
    #         rospy.loginfo("Obstacle detected within 1 meter! Stopping the robot.")
    #         self.stop_robot()
    #         return  # Exit the function early to immediately stop the robot

    #     clusters = self.euclidean_clustering(filtered_ranges, angle_increment, distance_threshold)

    #     min_size = 3  # Minimum number of points in a cluster to be considered a pedestrian
    #     max_size = 10  # Maximum number of points in a cluster to be considered a pedestrian
    #     pedestrian_clusters = self.detect_pedestrians(clusters, min_size, max_size, target_distance, distance_threshold)

    #     array_msg = Float32MultiArray()
    #     distances = []

    #     rospy.loginfo(len(self.distances))
    #     rospy.loginfo(len(pedestrian_clusters))

    #     for cluster in pedestrian_clusters:
    #         mean_angle = cluster[0]
    #         mean_distance = cluster[2]
    #         # Directly use mean_distance without cos adjustment
    #         for depth_distance in self.distances:
    #             if abs(mean_distance - depth_distance) < 0.2:
    #                 mean_distance = (mean_distance + depth_distance) / 2
    #                 break
    #         distances.append(mean_distance)

    #     #rospy.loginfo(distances)

    #     array_msg.data = distances if distances else []
    #     self.lidar_pub.publish(array_msg)
    #     self.rate.sleep()

    # def euclidean_clustering(self, ranges, angle_increment, distance_threshold):
    #     clusters = []
    #     cluster = []

    #     for i, range_value in enumerate(ranges):
    #         if np.isinf(range_value):
    #             continue
    #         angle = (i - len(ranges) // 2) * angle_increment  # Centering the angle

    #         if not cluster:
    #             cluster.append((angle, range_value))
    #         else:
    #             prev_angle, prev_range = cluster[-1]
    #             # Convert to Cartesian coordinates for distance calculation
    #             prev_x, prev_y = prev_range * np.cos(prev_angle), prev_range * np.sin(prev_angle)
    #             curr_x, curr_y = range_value * np.cos(angle), range_value * np.sin(angle)
    #             euclidean_dist = np.sqrt((curr_x - prev_x) ** 2 + (curr_y - prev_y) ** 2)
    #             if euclidean_dist < distance_threshold:
    #                 cluster.append((angle, range_value))
    #             else:
    #                 clusters.append(cluster)
    #                 cluster = [(angle, range_value)]

    #     if cluster:
    #         clusters.append(cluster)
    #     return clusters

    # def detect_pedestrians(self, clusters, min_size, max_size, target_distance, distance_threshold):
    #     pedestrian_clusters = []
    #     for cluster in clusters:
    #         if min_size < len(cluster) < max_size:
    #             angles = [point[0] for point in cluster]
    #             distances = [point[1] for point in cluster]
    #             mean_angle = np.mean(angles)
    #             mean_distance = np.mean(distances)
    #             if abs(mean_distance - target_distance) <= distance_threshold:
    #                 pedestrian_clusters.append((mean_angle, cluster, mean_distance))
    #     return pedestrian_clusters

    def detect_box_hog(self, image):
        hog = cv2.HOGDescriptor()
        hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

        (regions, weights) = hog.detectMultiScale(image, winStride=(4,4), padding=(8,8), scale = 1.05)
        
        rects = np.array([[x, y, x + w, y + h] for (x, y, w, h) in regions])
        pick = non_max_suppression(rects, probs=None, overlapThresh = 0.65)

        self.boxes = []
        for(x, y, x2, y2) in pick:
            self.boxes.append((x, y , abs(x2-x), abs(y2-y)))

    def detect_box_cascade(self, image):
        body=cv2.CascadeClassifier('/home/hugoc/RO50_project_ws/src/camera/scripts/haarcascade_fullbody.xml')
        detection=body.detectMultiScale(image,scaleFactor=1.05,minSize=(50,50))
        self.boxes = [box for box in detection]

    def detect_box(self, image):
        height, width, channels = image.shape

        # Detecting objects
        blob = cv2.dnn.blobFromImage(image, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
        self.net.setInput(blob)
        outs = self.net.forward(self.output_layers)

        # Showing information on the screen
        class_ids = []
        confidences = []
        self.boxes = []
        for out in outs:
            for detection in out:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                if confidence > 0.15 and class_id == 0:  # Only detecting humans (class_id == 0 for person)
                    center_x = int(detection[0] * width)
                    center_y = int(detection[1] * height)
                    w = int(detection[2] * width)
                    h = int(detection[3] * height)

                    # Rectangle coordinates
                    x = int(center_x - w / 2)
                    y = int(center_y - h / 2)

                    self.boxes.append([x, y, w, h])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)

        indexes = cv2.dnn.NMSBoxes(self.boxes, confidences, 0.5, 0.4)
        box_temp = []
        for i in range(len(self.boxes)):
            if i in indexes and w > 5 and h > 5 and y > 0 and x > 0:
                box_temp.append(self.boxes[i])
        self.boxes = box_temp        

    def detect_humans_distance(self):
        distance = []
        for i in range(len(self.boxes)):
            x, y, w, h = self.boxes[i]
            array = self.depth_image[y:y+h, x:x+w].flatten()
            
            # Ensure pixel_index is within bounds
            pixel_index = min(int(w * h * 0.1), len(array) - 1)
            
            if len(array) <= 0:
                distance.append(100)
                continue
            
            smallest_values = np.partition(array, pixel_index)[:pixel_index]
            
            if len(smallest_values) <= 0:
                distance.append(100)
                continue
            
            smallest_values_sorted = np.sort(smallest_values)
            distance.append(smallest_values_sorted[-1])
        
        return distance

    def draw_boxes(self, image, distances):
        for i in range(len(self.boxes)):
            x, y, w, h = self.boxes[i]
            color = (0, 255, 0)  # Green color for bounding box
            cv2.rectangle(image, (x, y), (x + w, y + h), color, 2)
            cv2.putText(image, "human", (x, y + 30), cv2.FONT_HERSHEY_PLAIN, 2, color, 2)
        return image

    def detect_humans(self, image):
        self.detect_box(image)
        self.distances = self.detect_humans_distance()
        #boxes_image = self.draw_boxes(image, distances)
        #self.save_image(boxes_image, "image_box_yolo.png")
        rospy.loginfo("yolo")
        #self.send_human_topic(self.distances)
        self.processing_image = False

    def stop_robot(self):
        start_time = rospy.get_time()

        # Send msg for 1 second to ensure the robot receives the information
        while not rospy.is_shutdown() and rospy.get_time() - start_time < 0.5:
            brake_trigger = 1
            self.brake_pub.publish(brake_trigger)
            rospy.loginfo("Emergency brake signal sent: %d", brake_trigger)
            self.rate.sleep()
    
    def send_human_topic(self, distances):
        array_msg = Float32MultiArray()
        if(len(distances) > 0):
            array_msg.data = distances
        else:
            array_msg.data = []

        self.human_pub.publish(array_msg)
        self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('obstacle_and_pedestrian_detector', anonymous=True)
    detector = ObstacleDetector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
    cv2.destroyAllWindows()