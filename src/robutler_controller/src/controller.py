#!/usr/bin/env python3
import rospy
import std_msgs.msg
import geometry_msgs.msg
from robutler_controller.msg import State
import numpy as np

class Controller:
    def __init__(self):
        rospy.init_node('controller', anonymous=True)
        self.state = State()
        self.sub = rospy.Subscriber('/amcl_pose', geometry_msgs.msg.PoseWithCovarianceStamped, self.position_callback)
        self.pub = rospy.Publisher('/state', State, queue_size=10)
        rospy.spin()

    def predict_room(self, x, y):
        pass

    def calculate_confidence(self, cov):
        #calculate confidence based on covariance matrix of the pose of the turtlebot
        cov = np.array(cov)
        cov = cov.reshape(6,6)
        if cov[0,0] < 0.5 and cov[1,1] < 0.5:
            return True
        else:
            return False
        

    def position_callback(self, args):
        self.state.x = args.pose.pose.position.x
        self.state.y = args.pose.pose.position.y
        self.state.orientation = args.pose.pose.orientation.z
        #calculate confidence based on covariance matrix of the pose of the turtlebot
        self.state.room_id = -1
        self.pub.publish(self.state)
        if not self.calculate_confidence(args.pose.covariance):
            self.state.current_state = "lost"
        else:
            self.state.current_state = "idle"
            self.state.room_id = self.predict_room(self.state.x, self.state.y)



if __name__ == '__main__':
    try:
        Controller()
    except rospy.ROSInterruptException:
        pass