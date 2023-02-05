#!/usr/bin/env python3

import roslib
roslib.load_manifest('robutler_controller')
import rospy
import actionlib

from robutler_controller.msg import MoveRobutlerAction
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class MoveRobutlerServer:
  def __init__(self):
    self.server = actionlib.SimpleActionServer('move_robutler', MoveRobutlerAction, self.execute, False)
    self.server.start()

  def execute(self, goal):
    rospy.loginfo('Executing move robutler action with goal: %s' % goal)
    self.move_robutler_to((goal.x, goal.y))
    self.server.set_succeeded()


  def move_robutler_to(self, coordinates):

      client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
      client.wait_for_server()

      goal = MoveBaseGoal()
      goal.target_pose.header.frame_id = "map"
      goal.target_pose.header.stamp = rospy.Time.now()
      goal.target_pose.pose.position.x = coordinates[0]
      goal.target_pose.pose.position.y = coordinates[1]
      goal.target_pose.pose.position.z = -0.0010080863701236245
      

      goal.target_pose.pose.orientation.x = 0
      goal.target_pose.pose.orientation.y = 0
      goal.target_pose.pose.orientation.z = 0
      goal.target_pose.pose.orientation.w = 1


      client.send_goal(goal)
      wait = client.wait_for_result()
      if not wait:
          rospy.logerr("Action server not available!")
          rospy.signal_shutdown("Action server not available!")
      else:
          return client.get_result()


if __name__ == '__main__':
  rospy.init_node('move_robutler_server')
  server = MoveRobutlerServer()
  rospy.spin()