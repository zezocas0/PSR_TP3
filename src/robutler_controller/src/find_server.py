#!/usr/bin/env python3

import roslib
roslib.load_manifest('robutler_controller')
import rospy
import actionlib
from robutler_controller.msg import FindAction

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

    #TODO: Try to find objects in the room

    rospy.loginfo(f'Found {goal.objectType} in {goal.room}')



if __name__ == '__main__':
  rospy.init_node('find_server')
  server = FindServer()
  rospy.spin()