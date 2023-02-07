#!/usr/bin/env python3

import roslib
roslib.load_manifest('robutler_controller')
import rospy
import actionlib
import time
from functools import partial
from robutler_controller.msg import CountFeedback, CountResult
from robutler_vision.msg import DetectedColor
from geometry_msgs.msg import Twist
from robutler_controller.msg import CountAction
PI = 3.1415926535897

def rotate(speed):
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()

    angular_speed = speed*2*PI/360

    vel_msg.linear.x=0
    vel_msg.linear.y=0
    vel_msg.linear.z=0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0

    vel_msg.angular.z = angular_speed

    # rospy.loginfo("Rotating...")
    velocity_publisher.publish(vel_msg)

def object_callback(args, msg):
    rospy.loginfo(msg)


class CountServer:
  def __init__(self):
    self.server = actionlib.SimpleActionServer('count', CountAction, self.execute, False)
    self.sub = rospy.Subscriber('/vision/color_detection', DetectedColor, self.color_detected)
    self.server.start()
    self.color = None

  def color_detected(self, msg):
    self.color = msg

  def execute(self, goal):
    rospy.loginfo(f'Counting {goal.objectType} in {goal.room}')
    self.count(goal)
    self.server.set_succeeded()


  def count(self, goal):
    rospy.loginfo('Counting...')
    start = time.time()
    while True:
        rotate(50)
        try:
          print(goal.color, self.color.polygon)
          if goal.color == self.color.polygon:
                  feedback = CountFeedback()
                  feedback.foundAny = True
                  self.server.publish_feedback()
                  result = CountResult()
                  result.num += 1
                  rospy.loginfo(f'Found {goal.objectType} {goal.color} in {goal.room}')

          if time.time() - start > 10:
              rospy.loginfo(f'Could not count {goal.objectType} in {goal.room}')
              rotate(0)
              break
        except AttributeError:
          pass

if __name__ == '__main__':
  rospy.init_node('count_server')
  server = CountServer()
  rospy.spin()