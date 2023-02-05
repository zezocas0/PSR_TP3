#!/usr/bin/env python3

import roslib
roslib.load_manifest('robutler_controller')
import rospy
import actionlib

from robutler_controller.msg import MoveRobutlerAction

class MoveRobutlerServer:
  def __init__(self):
    self.server = actionlib.SimpleActionServer('move_robutler', MoveRobutlerAction, self.execute, False)
    self.server.start()

  def execute(self, goal):
    rospy.loginfo('Executing move robutler action with goal: %s' % goal)
    self.server.set_succeeded()


if __name__ == '__main__':
  rospy.init_node('move_robutler_server')
  server = MoveRobutlerServer()
  rospy.spin()