#!/usr/bin/env python3
# Import ROS libraries and messages
from functools import partial
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
#from robutler_vision.msg import DetectedColor


import time
import json
import numpy as np

# Import OpenCV libraries and tools
import cv2
from cv_bridge import CvBridge, CvBridgeError

def getLimits():
    min_b = rospy.get_param('~min_b', 0)
    min_g = rospy.get_param('~min_g', 0)
    min_r = rospy.get_param('~min_r', 0)

    max_b = rospy.get_param('~max_b', 255)
    max_g = rospy.get_param('~max_g', 255)
    max_r = rospy.get_param('~max_r', 255)

    min = np.array([min_b, min_g, min_r], np.uint8)
    max = np.array([max_b, max_g, max_r], np.uint8)

    return min, max

def process_image(image):

    centroids = []

    min, max = getLimits()

    image = cv2.inRange(image, min, max)

    # find contours in the binary image
    contours, hierarchy = cv2.findContours(image,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    for c in contours:
        # calculate moments for each contour
        M = cv2.moments(c)
        
        # calculate x,y coordinate of center
        cX = int(M["m10"] / M["m00"]) if M["m00"] > 0 else 0
        cY = int(M["m01"] / M["m00"]) if M["m00"] > 0 else 0

        centroids.append((cX, cY))

        cv2.circle(image, (cX, cY), 5, (255, 0, 0), -1)

    return image, centroids

# Define a function to show the image in an OpenCV Window
def show_image(img):
    cv2.imshow("Image Window", img)
    cv2.waitKey(1)

# Define the message to be published
def colors_message(centroid):
    return str(centroid[0]) + "," + str(centroid[1])

# Define a callback for the Image message
def image_callback(args, img_msg):
    bridge = args['bridge']
    pub = args['pub']

    # log some info about the image topic
    rospy.loginfo(img_msg.header)

    # Try to convert the ROS Image message to a CV2 Image
    try:
        cv_image = bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))

    # Show the converted image
    start = time.time()
    image, centroids = process_image(cv_image)
    print("--- %s seconds ---" % (time.time() - start))

    for centroid in centroids:
        pub.publish(colors_message(centroid))
    show_image(image)

def main():

    rospy.init_node('camera_listener_color_detection', anonymous=True)
    rospy.loginfo("Starting camera_listener_color_detection node")

    pub = rospy.Publisher('/vision/color_detection', String, queue_size=10)


    # Get ROS parameters
    camera_topic = rospy.get_param('~camera_topic', '/camera/rgb/image_raw')

    # Initialize the CvBridge class
    bridge = CvBridge()
    # Subscribe to the camera topic and set callback function
    sub_image = rospy.Subscriber(camera_topic, Image, partial(image_callback, {'bridge': bridge, 'pub': pub}))


    # Initialize an OpenCV Window
    cv2.namedWindow("Image Window", 1)



if __name__ == '__main__':
    main()

    while not rospy.is_shutdown():
        rospy.spin()