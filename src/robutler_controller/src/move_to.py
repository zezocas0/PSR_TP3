#! /usr/bin/env python3

import roslib
roslib.load_manifest('robutler_controller')
import rospy
import actionlib

from robutler_controller.msg import MoveRobutlerAction, MoveRobutlerGoal

if __name__ == '__main__':
    rospy.init_node('move_robutler_client')
    client = actionlib.SimpleActionClient('move_robutler', MoveRobutlerAction)
    client.wait_for_server()

    goal = MoveRobutlerGoal()
    # Fill in the goal here
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(5.0))