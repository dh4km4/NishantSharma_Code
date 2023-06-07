"""
Senior Design Group: SARBot
Author: Nishant Sharma

The Purpose of this script is to read the unfiltered coordinates of human locations, filter it, and 
mark it on RVIZ on a new topic; The script recognizes clusters of coordinates and merges into one coordinate
and increases its priority.
"""
#!/usr/bin/env python
import sys
import os
import rospy
import argparse
import message_filters
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from sensor_msgs.msg import Image as msg_Image
from sensor_msgs.msg import CameraInfo
import pyrealsense2 as rs2
from geometry_msgs.msg import Pose
import tf2_ros
import tf2_geometry_msgs
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

# The two global parameters can be changed to 
# The Number of coordinates that must be inside a cluster to be considered a critical cluster (Marked in Red)
CRITICAL_CLUSTER_THRESHOLD = 6
# The radius of the sphere in meteres, that is drawn around coordinates to create a cluster (any coordinates within
# this distance is considered to be part of the cluster
SPHERE_RADIUS = 2.0

"""
Global Helper function that computes the distance between two 3D tuples
"""
def dist(x, y):
    return pow((pow((x[0]-y[0]), 2) + pow((x[1]-y[1]), 2) + pow((x[2]-y[2]), 2)), 0.5)


class Filter:
    """
    Initializes a filter, which reads all the coordinates, creates clusters, and adds new 
    weighted coordinates in the marker array filtered_coordinate_array
    """
    def __init__(self, filename):
        self.filename = filename
        self.marker_arr = MarkerArray()
        self.old_marker_arr = MarkerArray()
        self.old_count=0
        self.count = 0
        self.MARKER_MAX = 200
        self.coordinates = []
        self.old_coordinates = []
        self.filtered_coordinates = []
        self.filter_pub = rospy.Publisher('filtered_coordinate_array', MarkerArray, queue_size=self.MARKER_MAX)
        self.old_pub = rospy.Publisher('coordinate_array', MarkerArray, queue_size=self.MARKER_MAX)

    """
    Reads the coordinate.txt file in the same directory, and stores all the coordinates
    """
    def read_coordinates(self):
        f = open(self.filename, "r")
        lines = f.readlines()
        self.count = 0
        for line in lines:
            line = line.split()
            self.coordinates.append(((float(line[0])), (float(line[1])), (float(line[2]))))
            self.old_coordinates.append(((float(line[0])), (float(line[1])), (float(line[2]))))
    
    """
    Debug function to print the coordinates
    """
    def print_coordinates(self):
        for coordinate in self.coordinates:
            print(coordinate[0], coordinate[1], coordinate[2])
    
    """
    filter_coordinates: Goes through the coordinate array in a double loop; 
    for each element in the array, all the other elements that is within the sphere_radius
    is added to a cluster; The clusters with more than one coordinate is merged. 
    The average coordinate position is taken for each cluster, and it is added to the 
    filtered_coordinates array, which stores the coordinate and the weight of the coordinate.
    
    """
    def filter_coordinates(self):
        clusters = []
        i=0
        while i < (len(self.coordinates)):
            cluster=[]
            cluster.append(self.coordinates[i])
            j=i+1
            while j < (len(self.coordinates)):
                if (dist(self.coordinates[i], self.coordinates[j]) < SPHERE_RADIUS):
                    cluster.append(self.coordinates[j])
                    del(self.coordinates[j])
                else:
                    j+=1
            clusters.append(cluster)
            del(self.coordinates[i])
        
        my_sum = 0
        for cluster in clusters:
            #print(cluster)
            #print(len(cluster))
            #my_sum+=len(cluster)
            x = 0
            y = 0
            z = 0
            count = 0
            for coordinate in cluster:
                count+=1
                x+=coordinate[0]
                y+=coordinate[1]
                z+=coordinate[2]
            self.filtered_coordinates.append((x/count, y/count, z/count, count))

    """
    The unfiltered coordinates and the filtered coordinates are added to two marker arrays called
    coordinate_array, and filtered_coordinate_array respectively. The filtered coordinate that have a priority
    higher than the critical_cluster_threshold parameter, are marked in red. The size of the filtered_coordinate is also 
    proportional to the critical_cluster_threshold parameter. 
    """
    def mark_coordinates(self):
        for coord in self.filtered_coordinates:
            marker = Marker()
            marker.header.frame_id = "/map"
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            scaling = coord[3]
            if scaling > 15:
                scaling=15
            marker.scale.x, marker.scale.y, marker.scale.z = (scaling*0.05,scaling*0.05,scaling*0.05)
            marker.pose.position.x = coord[0]
            marker.pose.position.y = coord[1]
            marker.pose.position.z = coord[2]
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            if (scaling > CRITICAL_CLUSTER_THRESHOLD):    
                marker.color.r, marker.color.g, marker.color.b, marker.color.a = (1.0, 0.0, 0.0, 1.0)
            else:
                marker.color.r, marker.color.g, marker.color.b, marker.color.a = (0.0, 0.0, 1.0, 1.0)
            if (self.count > self.MARKER_MAX):
                self.marker_arr.markers.pop(0)
            self.marker_arr.markers.append(marker)
            id = 0
            for m in self.marker_arr.markers:
                m.id = id
                id+=1
            self.count += 1

        for coord in self.old_coordinates:
            #print(self.old_count)
            marker = Marker()
            marker.header.frame_id = "/map"
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.scale.x, marker.scale.y, marker.scale.z = (0.05,0.05,0.05)
            marker.pose.position.x = coord[0]
            marker.pose.position.y = coord[1]
            marker.pose.position.z = coord[2]
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = (0.0, 1.0, 0.0, 1.0)
            if (self.old_count > self.MARKER_MAX):
                self.old_marker_arr.markers.pop(0)
            self.old_marker_arr.markers.append(marker)
            id = 0
            for m in self.old_marker_arr.markers:
                m.id = id
                id+=1
            self.old_count+=1
        
    """
    Constantly publish the filtered cooridinate and the unfiltered coordinate array
    to their respective topics, so they can be viewed in RVIZ
    """
    def publish(self):
        self.filter_pub.publish(self.marker_arr)
        self.old_pub.publish(self.old_marker_arr)

"""
main method
"""
def main():
    my_filter = Filter("coordinates.txt")
    my_filter.read_coordinates()
    my_filter.print_coordinates()
    my_filter.filter_coordinates()
    my_filter.mark_coordinates()
    while not rospy.is_shutdown():
        my_filter.publish()

"""
And so it begins...
"""
if __name__ == '__main__':
    rospy.init_node("filter_py", anonymous=True)
    main()

