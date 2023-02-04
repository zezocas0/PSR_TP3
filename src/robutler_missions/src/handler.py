#!/usr/bin/env python3
# Import ROS libraries and messages
from functools import partial
import rospy
from std_msgs.msg import String
from robutler_missions.msg import Request

def msg_callback(args, msg):
    action = msg.action
    room = msg.room
    object = msg.object

    rospy.loginfo("Received message: action: %s - room: %s - object: %s", msg.action, msg.room, msg.object)

def main():
    rospy.init_node('camera_listener', anonymous=True)
    rospy.loginfo("Starting camera_listener node")

    # Subscribe to the mission topic and set callback function
    callback = rospy.Subscriber("/mission/requested", Request, partial(msg_callback, {}))



if __name__ == '__main__':
    main()

    while not rospy.is_shutdown():
        rospy.spin()