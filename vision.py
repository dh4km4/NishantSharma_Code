#!/usr/bin/env python
"""
Senior Design Group: SARBot - Search and Rescue Robot
Author Nishant Sharma

The purpose of the script is to use the Intel Realsense depth camera D455
to get the color and depth images of the surrounding; Detect any human faces that it 
sees, and mark its location using a marker on RVIZ. The script runs concurrently with 
the navigation script, and updates a marker array which creates a layer of human location
coordinates on top of the SLAM map in RVIZ
"""
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
import dlib

dir_path = os.path.dirname(os.path.realpath(__file__)) + '/'
human_frontal_face_cascade = cv2.CascadeClassifier(dir_path + 'haarcascade_frontalface_default.xml')
hogFaceDetector = dlib.get_frontal_face_detector()
cnn_face_detector = dlib.cnn_face_detection_model_v1(dir_path + "mmod_human_face_detector.dat")

# Global Variable mode can be changed to use different pretrained ML models 
# mode = 0 - Aruco Markers ; mode = 1 - HaarCascade FaceDetection; mode = 2 - HOG FaceDetection; mode = 3 - Cnn FaceDetection
mode = 2

# Show Video Feed
video = False

# Print Added Coordinates
log = True

class ImageListener:
    """
    Initializes ImageListener which subscribes to the necessary topics
    Detects any human faces, transforms the 3D coordinate to the map frame
    and stores the marker in a coordinates.txt file, and publish it to a Markerarray
    called coordinate_array
    """
    def __init__(self, depth_image_topic, depth_info_topic, color_image_topic):
        self.bridge = CvBridge()
        self.sub_depth = rospy.Subscriber(depth_image_topic, msg_Image, self.imageDepthCallback)
        self.sub_info = rospy.Subscriber(depth_info_topic, CameraInfo, self.imageDepthInfoCallback)
        self.sub_color = rospy.Subscriber(color_image_topic, msg_Image, self.imageColorCallback)
        confidence_topic = depth_image_topic.replace('depth', 'confidence')
        self.sub_conf = rospy.Subscriber(confidence_topic, msg_Image, self.confidenceCallback)
        self.intrinsics = None
        self.pix = None
        self.pix_grade = None
        self.centers = []
        self.coordinates = []
        self.marker_arr = MarkerArray()
        self.count = 0
        self.MARKER_MAX = 200
        self.pub = rospy.Publisher('coordinate_array', MarkerArray, queue_size = self.MARKER_MAX)
        self.fourcc = cv2.VideoWriter_fourcc('M', 'J', 'P', 'G')
        self.out = cv2.VideoWriter('output.avi', self.fourcc, 20.0, (1280, 720)) 
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(60))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer) 
        self.image_call_back = False
    
    """
    DepthCallBack: the function is called when the depth topic recieves a new depth image from the 
    D455 Depth Camera; The depth image must be aligned to the color image (align_depth flag in the camera
    launch file) and the depth and camera must be in sync (enable_synch flag in the camera launch file).
    The Depth call back checks the variable centers to see if there are any detected human faces, and if there are
    it gets the depth of the human face, creates a 3D coordinate, transforms it, stores it in coordinates.txt, and 
    publishes it to the coordinate_array topic 
    """
    def imageDepthCallback(self, data):
        if (not self.image_call_back):
            return
        else:
            self.image_call_back = False
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
            self.pub.publish(self.marker_arr)
            if (self.centers):
                for i in self.centers:
                    if (not self.intrinsics is None):
                        pix = (i[0], i[1])
                        self.pix = pix
	                line = '\rDepth at pixel(%3d, %3d): %7.1f(mm).' % (pix[0], pix[1], cv_image[pix[1], pix[0]])
	                if self.intrinsics:
	                    depth = cv_image[pix[1], pix[0]]
	                    result = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [pix[0], pix[1]], depth)
	                    line += '  Coordinate: %8.2f %8.2f %8.2f.' % (result[2], -result[0], -result[1])
	                if (not self.pix_grade is None):
	                    line += ' Grade: %2d' % self.pix_grade
	                line += '\r'
                        # Print line for DEBUG

                        before_transform_coordinate = (float(result[2])/1000.0, -float(result[0])/1000.0, -float(result[1])/1000.0)
                        coordinate = self.transform_pose(before_transform_coordinate, "camera_aligned_depth_to_color_frame", "map")
                        add_new_coordinate = True # Change to True
                        
                        if (before_transform_coordinate[0] == 0 and before_transform_coordinate[1] == 0 and before_transform_coordinate[2] == 0):
                            add_new_coordinate = False
                        for coord in self.coordinates:
                            if ((abs(abs(coord[0]) - abs(coordinate[0])) < 0.1)
                            and (abs(abs(coord[1]) - abs(coordinate[1])) < 0.1)
                            and (abs(abs(coord[2]) - abs(coordinate[2])) < 0.1)):
                                add_new_coordinate = False
                        if add_new_coordinate:
                            self.coordinates.append(coordinate)
                            f = open(dir_path + "coordinates.txt", "w")
                            if (log):
                                print("writing new coordinate") 
                                print("Before Transform:- ", before_transform_coordinate[0], before_transform_coordinate[1], before_transform_coordinate[2]) 
                                print("After Transform:- ", coordinate[0], coordinate[1], coordinate[2])
                            self.add_marker(coordinate)
                            for coord in self.coordinates:
                                coord_str = str(coord[0]) + " " + str(coord[1]) + " " +  str(coord[2]) + "\n"
                                f.write(coord_str)
                            f.close()
        except CvBridgeError as e:
            print(e)
            return
        except ValueError as e:
            return

    """
    ImageColorCallBack: The funciton is called when a new color image is recieved in the color 
    image topic. ML models for face detection are run on the the color image frame, and it is stored 
    in a video file called output.avi. If the global flag video is set to true then the video feed is displayed 
    in real-time
    """
    def imageColorCallback(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding) 
        centers = []
        if mode == 0:
            (frame, corners) = self.aruco_detector(cv_image)
            try:
                for markerCorner in corners:
                    (x, y, w, h) = markerCorner.reshape((4,2))
                    x = (x[0], x[1])
                    y = (y[0], y[1])
                    w = (w[0], w[1])
                    h = (h[0], h[1])
                    centers.append(np.array([int((x[0]+w[0])/2), int((x[1]+w[1])/2)]))
                self.centers = centers
                if (video):
                    cv2.imshow("color camera aruco", frame)
                    cv2.waitKey(1)
                self.out.write(frame)
            except CvBridgeError as e:
                print(e)
                return
            except ValueError as e:
                return
        elif mode == 1:
            (frame, points) = self.human_detector_haar(cv_image)
            try:
                for (a,b,c,d) in points:
                    x = (a, b)
                    w = ((a+c), (b+d))
                    centers.append(np.array([int((x[0]+w[0])/2), int((x[1]+w[1])/2)]))
                self.centers = centers
                if (video):
                    cv2.imshow("color camera haarcascade", frame)
                    cv2.waitKey(1)
                self.out.write(frame)
            except CvBridgeError as e:
                print(e)
                return
            except ValueError as e:
                return
	elif mode == 2:
            (frame, points) = self.human_detector_hog(cv_image)
            try:
                for (a,b,c,d) in points:
                    x = (a, b)
                    w = ((a+c), (b+d))
                    centers.append(np.array([int((x[0]+w[0])/2), int((x[1]+w[1])/2)]))
                self.centers = centers 
                if (video):
                    cv2.imshow("color camera hog", frame)
                    cv2.waitKey(1)
                self.out.write(frame)
            except CvBridgeError as e:
                print(e)
                return
            except ValueError as e:
                return
	elif mode == 3:
            (frame, points) = self.human_detector_cnn(cv_image)
            try:
                for (a,b,c,d) in points:
                    x = (a, b)
                    w = ((a+c), (b+d))
                    centers.append(np.array([int((x[0]+w[0])/2), int((x[1]+w[1])/2)]))
                self.centers = centers
                if (video):
                    cv2.imshow("color camera cnn", frame)
                    cv2.waitKey(1)
                self.out.write(frame)
            except CvBridgeError as e:
                print(e)
                return
            except ValueError as e:
                return
        self.image_call_back = True

    """
    Stored the depth confidence which is needed to get the depth of a pixel in the depth callback
    """
    def confidenceCallback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
            grades = np.bitwise_and(cv_image >> 4, 0x0f)
            if (self.pix):
                self.pix_grade = grades[self.pix[1], self.pix[0]]
        except CvBridgeError as e:
            print(e)
            return

    """
    Stores the camera information about the depth image needed to get the depth of a pixel in the depth callback
    """
    def imageDepthInfoCallback(self, cameraInfo):
        try:
            if self.intrinsics:
                return
            self.intrinsics = rs2.intrinsics()
            self.intrinsics.width = cameraInfo.width
            self.intrinsics.height = cameraInfo.height
            self.intrinsics.ppx = cameraInfo.K[2]
            self.intrinsics.ppy = cameraInfo.K[5]
            self.intrinsics.fx = cameraInfo.K[0]
            self.intrinsics.fy = cameraInfo.K[4]
            if cameraInfo.distortion_model == 'plumb_bob':
                self.intrinsics.model = rs2.distortion.brown_conrady
            elif cameraInfo.distortion_model == 'equidistant':
                self.intrinsics.model = rs2.distortion.kannala_brandt4
            self.intrinsics.coeffs = [i for i in cameraInfo.D]
        except CvBridgeError as e:
            print(e)
            return
    
    """
    OpenCV's aruco marker detector is used to detect a 4x4 Aruco Marker; The corners and the updated 
    frame with a rextangle drawn around the aruco markers are returned 
    """
    def aruco_detector(self, frame):
        arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        arucoParams = cv2.aruco.DetectorParameters_create()
        (corners, ids, rejected) = cv2.aruco.detectMarkers(frame, arucoDict, parameters=arucoParams)
        
        for markerCorner in corners:
            (x, y, w, h) = markerCorner.reshape((4,2))
            x = (x[0], x[1])
            y = (y[0], y[1])
            w = (w[0], w[1])
            h = (h[0], h[1])
            cv2.line(frame, x, y, (0, 0, 255), 2)
            cv2.line(frame, x, h, (0, 0, 255), 2)
            cv2.line(frame, y, w, (0, 0, 255), 2)
            cv2.line(frame, w, h, (0, 0, 255), 2)
        return (frame, corners)

    """
    OpenCV's Haarcascade classifier is used to detect Human Faces; The corners and the updated 
    frame with a rextangle drawn around the Human Faces are returned 
    """
    def human_detector_haar(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        frontal_faces = human_frontal_face_cascade.detectMultiScale3(gray, scaleFactor=1.05, minNeighbors=5, minSize=(30,30), flags=cv2.CASCADE_SCALE_IMAGE, outputRejectLevels=True)
        rect = frontal_faces[0]
        neighbors = frontal_faces[1]
        weights = frontal_faces[2]
        corners = []
        for (x,y,w,h) in rect:
            cv2.rectangle(frame,(x,y),(x+w,y+h),(255,0,255),2)
            corners.append((x,y,w,h))
            # print("rect: ", rect, "weight: ", weights)
        return (frame, corners)

    """
    dlib's pretrained model that uses Histogram of Gradients is used to detect Human Faces; The corners and the updated 
    frame with a rextangle drawn around the Human Faces are returned 
    """
    def human_detector_hog(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	faces = hogFaceDetector(gray, 1)
        corners = []
	for (i, rect) in enumerate(faces):
            x = rect.left()
            y = rect.top()
            w = rect.right() - x
            h = rect.bottom() - y
            cv2.rectangle(frame,(x,y), (x+w, y+h), (255, 0, 0), 2)
            corners.append((x,y,w,h))
        return (frame, corners)
	
    """
    dlib's pretrained model that uses cnn is used to detect Human Faces; The corners and the updated 
    frame with a rextangle drawn around the Human Faces are returned; NEEDS A GPU to work
    """
    def human_detector_cnn(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	faces = cnn_face_detector(gray, 1)
        corners = []
	for faceRect in faces:
	    rect = faceRect.rect
	    x = rect.left()
	    y = rect.top()
	    w = rect.bottom() - x
	    h = rect.bottom() - y
	    cv2.rectangle(frame,(x,y), (x+w, y+h), (255, 0, 0), 2)
	    corners.append((x,y,w,h))
        return (frame, corners)
    
    """
    Function transforms a coordinate 3D tuple from one frame to another; Returns a 3D coordinate tuple
    """
    def transform_pose(self, coordinate, from_frame, to_frame):
        input_pose = Pose()
        input_pose.position.x = coordinate[0]
        input_pose.position.y = coordinate[1]
        input_pose.position.z = coordinate[2]
        input_pose.orientation.x = 0.0
        input_pose.orientation.y = 0.0
        input_pose.orientation.z = 0.0
        input_pose.orientation.w = 0.0
        pose_stamped = tf2_geometry_msgs.PoseStamped()
        pose_stamped.pose = input_pose
        pose_stamped.header.frame_id = from_frame
        pose_stamped.header.stamp = rospy.Time(0)
        try:
            output_pose_stamped = self.tf_buffer.transform(pose_stamped, to_frame, rospy.Duration(1))
            return (output_pose_stamped.pose.position.x, output_pose_stamped.pose.position.y, output_pose_stamped.pose.position.z)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            raise
    
    """
    Given a 3D tuple, a marker is created and added to the marker array stored in self.marker_arr; The marker array is 
    constantly published to the coordinate_array topic; This is done to visualize the markers in RVIZ
    """
    def add_marker(self, coordinate):
        marker = Marker()
        marker.header.frame_id = "/map"
        marker.type = marker.SPHERE
	marker.action = marker.ADD
        marker.scale.x, marker.scale.y, marker.scale.z = (0.05, 0.05, 0.05)
        marker.pose.position.x = float(coordinate[0])
        marker.pose.position.y = float(coordinate[1])
        marker.pose.position.z = float(coordinate[2])
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0 
        marker.pose.orientation.w = 1.0
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = (1.0, 0.0, 0.0, 1.0)
        if (self.count > self.MARKER_MAX):
            self.marker_arr.markers.pop(0)
        self.marker_arr.markers.append(marker)
        id = 0
        for m in self.marker_arr.markers:
            m.id = id
            id+=1
        self.count += 1

"""
Initialize the Image listener, and input the name of the topics that it requires; The aligned depth image, 
color image, depth confidence, and aligned depth camera info
"""
def main():
    f = open(dir_path + "coordinates.txt", "w")
    f.close()
    depth_image_topic = '/camera/aligned_depth_to_color/image_raw'
    depth_info_topic = '/camera/aligned_depth_to_color/camera_info'
    color_image_topic = '/camera/color/image_raw'
    depth_color_aligned_topic = '/camera/aligned_depth_to_color/image_raw'
    print ('Vision - Detection and Marking Coordinates')
    listener = ImageListener(depth_image_topic, depth_info_topic, color_image_topic)
    rospy.spin()

"""
And so it begins...
"""
if __name__ == '__main__':	
    rospy.init_node('vision_py', anonymous=True)
    main()
