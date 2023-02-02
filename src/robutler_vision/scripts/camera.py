#!/usr/bin/env python3
# Import ROS libraries and messages
import rospy
from sensor_msgs.msg import Image

import YOLO.yolo as yolo
# Import OpenCV libraries and tools
import cv2
from cv_bridge import CvBridge, CvBridgeError

rospy.init_node('camera_listener', anonymous=True)
rospy.loginfo("Starting camera_listener node")

# Initialize the CvBridge class
bridge = CvBridge()

# YOLO parameters
net = None
classes = None

# Define a function to show the image in an OpenCV Window
def show_image(img):
    cv2.imshow("Image Window", img)
    cv2.waitKey(1)

# Define a callback for the Image message
def image_callback(img_msg):
    # log some info about the image topic
    rospy.loginfo(img_msg.header)

    # Try to convert the ROS Image message to a CV2 Image
    try:
        cv_image = bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))

    # Show the converted image
    # show_image(cv_image)
    if net is None or classes is None:
        return
    objects = yolo.detect(cv_image, net, classes)
    
    # Show the image
    show_image(objects)


def main():
    global net, classes
    sub_image = rospy.Subscriber("/camera/rgb/image_raw", Image, image_callback)

    # Load YOLO
    net, classes = yolo.load_model("YOLO/yolov3.cfg", "YOLO/yolov3-tiny.weights", "YOLO/yolov3.txt")

    print(net)
    print(classes)

    # Initialize an OpenCV Window
    cv2.namedWindow("Image Window", 1)



if __name__ == '__main__':
    main()

    while not rospy.is_shutdown():
        rospy.spin()