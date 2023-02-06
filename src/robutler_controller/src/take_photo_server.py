#!/usr/bin/env python3

import roslib
roslib.load_manifest('robutler_controller')
import rospy
import actionlib

from sensor_msgs.msg import Image
import cv2
from robutler_controller.msg import TakePhotoAction
from cv_bridge import CvBridge, CvBridgeError

class TakePhotoServer:
  def __init__(self):
    self.server = actionlib.SimpleActionServer('take_photo', TakePhotoAction, self.execute, False)
    self.server.start()

  def execute(self, goal):
    rospy.loginfo('Executing take a photo in room: %s' % goal.room)
    self.take_photo(goal)
    self.server.set_succeeded()


  def take_photo(self, goal):
    # Initialize the CvBridge class
    bridge = CvBridge()

    rospy.loginfo('Taking Photo...')
    image_msg = rospy.wait_for_message('/camera/rgb/image_raw', Image)
    rospy.loginfo('Photo Taken!')

    try:
        image = bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))

    cv2.imwrite(f'{goal.room}:{goal.stamp}.png', image)

    rospy.loginfo(f'Saved image to ~/.ros/{goal.room}:{goal.stamp}.png')



if __name__ == '__main__':
  rospy.init_node('take_photo_server')
  server = TakePhotoServer()
  rospy.spin()