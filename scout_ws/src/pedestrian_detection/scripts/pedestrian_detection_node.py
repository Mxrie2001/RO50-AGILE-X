#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Int32
import numpy as np

marker_pub = None
brake_pub = None
rate = None

def euclidean_clustering(ranges, angle_increment, distance_threshold):
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

def detect_pedestrians(clusters, min_size, max_size, target_distance, distance_threshold):
    pedestrian_clusters = []
    for cluster in clusters:
        if min_size < len(cluster) < max_size:
            mean_distance = np.mean([point[1] for point in cluster])
            if abs(mean_distance - target_distance) <= distance_threshold:
                pedestrian_clusters.append((cluster, mean_distance))
    return pedestrian_clusters

def publish_markers(pedestrian_clusters):
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

    marker_pub.publish(marker_array)

def callback(data):
    ranges = np.array(data.ranges)
    angle_increment = data.angle_increment
    distance_threshold = 0.2  # Distance threshold for clustering in meters
    target_distance = 1.0     # Target distance for pedestrian detection
    clusters = euclidean_clustering(ranges, angle_increment, distance_threshold)
    
    min_size = 5   # Minimum number of points in a cluster to be considered a pedestrian
    max_size = 50  # Maximum number of points in a cluster to be considered a pedestrian
    pedestrian_clusters = detect_pedestrians(clusters, min_size, max_size, target_distance, distance_threshold)
    
    if pedestrian_clusters:
        rospy.loginfo("Pedestrians detected: %d", len(pedestrian_clusters))
        publish_markers(pedestrian_clusters)
        stop_robot()

def stop_robot():
    start_time = rospy.get_time()

    # Send msg for 1 second to ensure the robot receives the information
    while not rospy.is_shutdown() and rospy.get_time() - start_time < 1:
        brake_trigger = 1
        brake_pub.publish(brake_trigger)
        rospy.loginfo("Emergency brake signal sent: %d", brake_trigger)

        rate.sleep()

def pedestrian_detection_node():
    global marker_pub, brake_pub, rate
    rospy.init_node('pedestrian_detection_node', anonymous=True)
    rate = rospy.Rate(10)
    rospy.Subscriber('/scan', LaserScan, callback)
    brake_pub = rospy.Publisher('/emergency_brake', Int32, queue_size=1)
    marker_pub = rospy.Publisher('pedestrian_markers', MarkerArray, queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    try:
        pedestrian_detection_node()
    except rospy.ROSInterruptException:
        pass
