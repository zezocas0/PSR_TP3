#!/usr/bin/env python3
import rospy
import std_msgs.msg
import geometry_msgs.msg
from robutler_controller.msg import State

class Controller:
    def __init__(self):
        rospy.init_node('controller', anonymous=True)
        self.state = State()
        self.sub = rospy.Subscriber('/amcl_pose', geometry_msgs.msg.PoseWithCovarianceStamped, self.position_callback)
        self.pub = rospy.Publisher('/state', State, queue_size=10)
        self.state = State()
        rospy.spin()

    def position_callback(self, args):
        self.state.x = args.pose.pose.position.x
        self.state.y = args.pose.pose.position.y
        self.state.orientation = args.pose.pose.orientation.z
        #calculate confidence based on covariance matrix of the pose of the turtlebot
        self.state.position_confidence = 0
        self.state.current_state = "idle"
        self.pub.publish(self.state)
    


if __name__ == '__main__':
    try:
        Controller()
    except rospy.ROSInterruptException:
        pass