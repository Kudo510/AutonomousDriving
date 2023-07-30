import rospy
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
import std_msgs.msg
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import sensor_msgs.point_cloud2 as pcl2
import pcl_ros
from collections import namedtuple
import ctypes
import math
from geometry_msgs.msg import Point


K = []
contours = []
new_width = 1280  # Adjust the width as desired
new_height = 720  # Adjust the height as desired

# Callback function to process the received messages
def segmentation_callback(segmentation_msg):
    global contours
    # Create a publisher to publish the received messages
    bridge = CvBridge()
    try:
        # Convert the received segmentation, RGB, and depth image messages to OpenCV format
        segmentation_image = bridge.imgmsg_to_cv2(segmentation_msg, desired_encoding="passthrough")

        # Increase the resolution of the segmentation image
        segmentation_image = cv2.resize(segmentation_image, (new_width, new_height))

        # Define the blue color range for thresholding
        lower_blue = np.array([100, 0, 0], dtype=np.uint8)
        upper_blue = np.array([255, 50, 50], dtype=np.uint8)

        # Threshold the image to create a binary mask for the blue objects
        blue_mask = cv2.inRange(segmentation_image, lower_blue, upper_blue)

        # Find contours in the binary mask
        contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))
        return


def depth_callback(depth_msg):
    global contours
    if len(K) != 0:
        # Create a publisher to publish the received messages
        bridge = CvBridge()
        try:
            depth_image = bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
            if len(contours) != 0:
                points = convert_contours_to_points(contours, depth_image)
                header = std_msgs.msg.Header()
                header.stamp = rospy.Time.now()
                header.frame_id = 'body'
                point_cloud_msg = pcl2.create_cloud_xyz32(header, points)
                point_cloud_publisher.publish(point_cloud_msg)


        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
            return


def K_callback(camera_msg):
    global K
    # Access the K field of the message
    K = np.array(camera_msg.K).reshape((3, 3))


def convert_contours_to_points(contours, depth_image):
    global K
    points = []
    # Iterate over the contours and convert each point to 3D
    for contour in contours:
        for point in contour:
            # pixel coordinate for each point in contours
            u = int(point[0][0] * 320 / new_width)
            v = int(point[0][1] * 240 / new_height)
            depth = depth_image[v, u]

            # convert pixel coordinate to camera coordinate
            K_inv = np.linalg.inv(K)
            uv = np.array([u * depth, v * depth, depth])
            xyz = K_inv @ uv
            #  Convert from camera frame to body frame and convert from mm to meter
            x_3d = xyz[2] / 1000
            y_3d = -xyz[0] / 1000
            z_3d = xyz[1] / 1000

            if (x_3d != 0.0 or y_3d != 0.0 or z_3d != 0.0):
                max_range = 25
                if (x_3d < max_range):
                    points.append([x_3d, y_3d, z_3d])
    return points


# Initialize the ROS node
rospy.init_node("cars_detection")

# Create a publisher to publish the received messages
point_cloud_publisher = rospy.Publisher("point_cloud_position_of_cars", PointCloud2, queue_size=5)

# Subscribe to the segmentation, depth, and camera info topics
K_topic = "/unity_ros/OurCar/Sensors/DepthCamera/camera_info"
segmentation_topic = "/unity_ros/OurCar/Sensors/SemanticCamera/image_raw"
depth_topic = "/unity_ros/OurCar/Sensors/DepthCamera/image_raw"

rospy.Subscriber(K_topic, CameraInfo, K_callback)
rospy.Subscriber(segmentation_topic, Image, segmentation_callback)
rospy.Subscriber(depth_topic, Image, depth_callback)


# Spin ROS
rospy.spin()
