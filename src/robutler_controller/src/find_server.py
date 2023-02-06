#!/usr/bin/env python3

import roslib
roslib.load_manifest('robutler_controller')
import rospy
import actionlib
from functools import partial
from robutler_controller.msg import FindAction
from robutler_vision.msg import DetectedObject
from geometry_msgs.msg import Twist
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


class FindServer:
  def __init__(self):
    self.server = actionlib.SimpleActionServer('find', FindAction, self.execute, False)
    self.server.start()

  def execute(self, goal):
    rospy.loginfo(f'Executing find {goal.objectType} in {goal.room}')
    self.find(goal)
    self.server.set_succeeded()


  def find(self, goal):
    rospy.loginfo('Finding...')

    while True:
        rotate(50)
        try:
            obj = rospy.wait_for_message("/vision/object_detection", DetectedObject, timeout=0.1)
            rospy.loginfo(f'Found {obj}')
            if goal.objectType == obj:
                break
        except rospy.ROSException:
            continue

    rospy.loginfo(f'Found {goal.objectType} in {goal.room}')



if __name__ == '__main__':
  rospy.init_node('find_server')
  server = FindServer()
  rospy.spin()