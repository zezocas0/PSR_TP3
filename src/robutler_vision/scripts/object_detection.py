#!/usr/bin/env python3
# Import ROS libraries and messages
from functools import partial
import json
import time
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String

# Import YOLO model
from YOLO.yolo import yolo as YOLO

# Import OpenCV libraries and tools
import cv2
from cv_bridge import CvBridge, CvBridgeError

# Define a function to show the image in an OpenCV Window
def show_image(img):
    cv2.imshow("Image Window", img)
    cv2.waitKey(1)

# Define the message to be published
def objects_message(objects):
    return json.dumps([
        { "class": obj[0], "confidence": float(obj[1][0]), 
        "left": int(obj[2][0]), "top": int(obj[2][1]), 
        "width": int(obj[2][2]), "height": int(obj[2][3]) } 
    for obj in objects])


# Define a callback for the Image message
def image_callback(args, img_msg):
    yolo = args['yolo']
    bridge = args['bridge']
    pub = args['pub']

    # log some info about the image topic
    rospy.loginfo(img_msg.header)

    # Try to convert the ROS Image message to a CV2 Image
    try:
        cv_image = bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))

    if yolo.model is None or yolo.net is None or yolo.classes is None:
        print("YOLO not loaded")
        return

    start = time.time()
    image, objects = yolo.detect(cv_image)
    print("--- %s seconds ---" % (time.time() - start))
    
    # Show the image
    pub.publish(objects_message(objects))
    show_image(image)


def main():
    rospy.init_node('camera_listener', anonymous=True)
    rospy.loginfo("Starting camera_listener node")

    pub = rospy.Publisher('/vision/object_detection', String, queue_size=10)

    # Initialize the CvBridge class
    bridge = CvBridge()

    # Load YOLO
    yolo = YOLO(rospy.get_param("~config_file"), rospy.get_param("~weights_file"), rospy.get_param("~coco_names"))

    # Subscribe to the camera topic and set callback function
    sub_image = rospy.Subscriber("/camera/rgb/image_raw", Image, partial(image_callback, {'yolo': yolo, 'bridge': bridge, 'pub': pub}))


    # Initialize an OpenCV Window
    cv2.namedWindow("Image Window", 1)



if __name__ == '__main__':
    main()

    while not rospy.is_shutdown():
        rospy.spin()