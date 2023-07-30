import numpy as np
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import Float32
from std_msgs.msg import String
from object_detection_pkg.msg import DepthAndColor


# Global variables to store the segmentation, RGB, and depth images
segmentation_image = None
depth_image = None
rgb_image_rgb = None
depth=None
dominant_color=None

# Callback function to process the received segmentation message
def segmentation_callback(segmentation_msg):
    global segmentation_image
    bridge = CvBridge()
    try:
        # Convert the received segmentation image message to OpenCV format
        segmentation_image = bridge.imgmsg_to_cv2(segmentation_msg, desired_encoding="passthrough")
        # Convert the segmentation image to RGB
        segmentation_image = cv2.cvtColor(segmentation_image, cv2.COLOR_BGR2RGB)
        # Increase the resolution of the segmentation image
        segmentation_image = cv2.resize(segmentation_image, (new_width, new_height))
        #take the middle part of the image only to detect traffic light
        segmentation_image = segmentation_image[:,360:720]
        

    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))

# Callback function to process the received depth image message
def depth_callback(depth_msg):
    global depth_image
    global depth
    bridge = CvBridge()
    try:
        # Convert the received depth image message to OpenCV format
        depth_image = bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")

        # Check if the segmentation are available
        if segmentation_image is not None:
            # Threshold the semantic segmentation image to create a binary mask
            aqua_color = np.array([0, 255, 255])
            threshold = 200  # Adjust this threshold value to segment your object accurately
            mask = np.linalg.norm(segmentation_image - aqua_color, axis=2) < threshold
            mask = mask.astype(np.uint8) * 255
            # Find contours in the binary mask
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if len(contours) > 0:
                # Select the contour that corresponds to the object
                selected_contour = max(contours, key=cv2.contourArea)
                # Check if the contour area is greater than 50
                if cv2.contourArea(selected_contour) > 50:
                    # Find the bounding box coordinates of the contour
                    x, y, w, h = cv2.boundingRect(selected_contour)

                    # Calculate the center point of the bounding box
                    center_x = x +360 + w // 2
                    center_y = y + h // 2

                    #convert position b
                    original_center_x = int(center_x * 320 / new_width)
                    original_center_y = int(center_y * 240 / new_height)

                    depth = depth_image[original_center_y, original_center_x]   #depth value here is in mm

    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))

# Callback function to process the received RGB image message
def rgb_callback(rgb_msg):
    global rgb_image_rgb
    global dominant_color
    global depth
    bridge = CvBridge()
    try:
        # Convert the received RGB image message to OpenCV format
        image = bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="passthrough")
        # Increase the resolution of the RGB image
        rgb_image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        rgb_image_rgb = cv2.resize(rgb_image_rgb, (new_width, new_height))
        rgb_image_rgb = rgb_image_rgb[:,360:720]

        # Check if the segmentation image is available
        if segmentation_image is not None:
            # Threshold the semantic segmentation image to create a binary mask
            aqua_color = np.array([0, 255, 255])
            threshold = 200  # Adjust this threshold value to segment your object accurately
            mask = (np.linalg.norm(segmentation_image - aqua_color, axis=2) < threshold)
            mask = mask.astype(np.uint8) * 255

            # Find contours in the binary mask
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if len(contours) > 0:
                # Select the contour that corresponds to the object
                selected_contour = max(contours, key=cv2.contourArea)
                # Check if the contour area is greater than 50
                if cv2.contourArea(selected_contour) > 50:
                    # Create a mask with the same shape as the RGB image and fill the contour region
                    new_mask = np.zeros_like(rgb_image_rgb)
                    cv2.drawContours(new_mask, [selected_contour], 0, (255, 255, 255), cv2.FILLED)

                    # Apply the new mask to the RGB image to crop out the region of your object
                    cropped_image_rgb = cv2.bitwise_and(rgb_image_rgb, new_mask)

                    # Find the bounding box coordinates of the contour
                    x, y, w, h = cv2.boundingRect(selected_contour)

                    # Draw the bounding box on the original RGB image
                    bounding_box_image = cropped_image_rgb.copy()
                    cv2.rectangle(bounding_box_image, (x, y), (x + w, y + h), (0, 255, 0), 2)

                    # Crop out the region within the bounding box
                    cropped_image_rgb = cropped_image_rgb[y:y+h, x:x+w]

                    # Convert the cropped image to the HSV color space
                    hsv_image = cv2.cvtColor(cropped_image_rgb, cv2.COLOR_BGR2HSV)

                    # Define the color ranges for red, yellow, and green
                    red_lower = np.array([0, 100, 100])
                    red_upper = np.array([10, 255, 255])

                    yellow_lower = np.array([25, 100, 100])
                    yellow_upper = np.array([35, 255, 255])

                    green_lower = np.array([50, 100, 100])
                    green_upper = np.array([70, 255, 255])

                    # Apply color thresholding to detect red, yellow, and green regions
                    red_mask = cv2.inRange(hsv_image, red_lower, red_upper)
                    yellow_mask = cv2.inRange(hsv_image, yellow_lower, yellow_upper)
                    green_mask = cv2.inRange(hsv_image, green_lower, green_upper)

                    # Count the number of non-zero pixels in each mask
                    red_pixel_count = cv2.countNonZero(red_mask)
                    yellow_pixel_count = cv2.countNonZero(yellow_mask)
                    green_pixel_count = cv2.countNonZero(green_mask)

                    # Determine the dominant color based on the pixel counts 
                    if red_pixel_count > yellow_pixel_count and red_pixel_count > green_pixel_count:
                        dominant_color = "Red"
                    else:
                        dominant_color = "Green"
                        
                    # Publish the dominant color as a String message
                    msg = DepthAndColor()
                    msg.depth = depth
                    msg.dominant_color = dominant_color
                    pub.publish(msg)

    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))

# Initialize the ROS node
rospy.init_node("traffic_light_status")

# # Create a publisher to publish the depth value
# depth_publisher = rospy.Publisher("traffic_light_position", Float32, queue_size=10)
# Create a publisher to publish the processed dominant color
pub = rospy.Publisher("traffic_light_status", DepthAndColor, queue_size=1)

# Set the desired resolution for the images
new_width = 1280  # Adjust the width as desired
new_height = 720  # Adjust the height as desired

# Subscribe to the segmentation, RGB, and depth image topics
segmentation_topic = "/unity_ros/OurCar/Sensors/SemanticCamera/image_raw"
depth_topic = "/unity_ros/OurCar/Sensors/DepthCamera/image_raw"
rgb_topic = "/unity_ros/OurCar/Sensors/RGBCameraRight/image_raw"


rospy.Subscriber(rgb_topic, Image, rgb_callback)
rospy.Subscriber(segmentation_topic, Image, segmentation_callback)
rospy.Subscriber(depth_topic, Image, depth_callback)


# Spin ROS
rospy.spin()


