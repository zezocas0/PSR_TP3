#!/usr/bin/env python3
# Import ROS libraries and messages
from functools import partial
import rospy
import actionlib

from datetime import datetime
from std_msgs.msg import String
from robutler_missions.msg import Request
from robutler_controller.msg import MoveRobutlerAction, MoveRobutlerGoal, TakePhotoAction, TakePhotoGoal, FindAction, FindGoal

# from missions import Actions

'''TODO: LATER IMPORT THIS FROM missions.py FILE'''
class Room:
    def __init__(self, name: str, coordinates: list):
        self.name = name
        self.coordinates = coordinates
    def get_name(self):
        return self.name
    def get_id(self):
        return self.id
    def get_coordinates(self):
        return self.coordinates

class Object:
    def __init__(self, name: str, location: Room = None, count: int = 0):
        self.name = name
        self.location = location
        self.count = count
    def get_name(self):
        return self.name
    def get_location(self):
        return self.location
    def get_count(self):
        return self.count
    def set_location(self, location: Room):
        self.location = location
    def set_count(self, count: int):
        self.count = count
        
class Actions:
    def __init__(self):
        self.rooms = []
        self.objects = []

    def add_room(self, name, coordinates):
        room = Room(name, coordinates)
        self.rooms.append(room)

    def add_object(self, name, location = None, count = 0):
        object = Object(name, location, count)
        self.objects.append(object)

    def go_to_room(self, room: Room):
        rospy.loginfo("Going to room: %s", room.get_name())

        client = actionlib.SimpleActionClient('move_robutler', MoveRobutlerAction)
        client.wait_for_server()

        goal = MoveRobutlerGoal()
        # Fill in the goal here
        coordinates = room.get_coordinates()
        goal.x = coordinates[0]
        goal.y = coordinates[1]
        client.send_goal(goal)
        client.wait_for_result()
    
    def photo(self, room: Room):
        rospy.loginfo("Taking photo in room: %s", room.get_name())

        self.go_to_room(room)

        rospy.loginfo("Reached the room...")
        client = actionlib.SimpleActionClient('take_photo', TakePhotoAction)
        client.wait_for_server()

        goal = TakePhotoGoal()
        goal.room = room.get_name()
        goal.stamp = rospy.Time.now()
        client.send_goal(goal)
        client.wait_for_result()

    def count(self, room: Room, object: Object):
        rospy.loginfo("Counting %s in room: %s", object.get_name(), room.get_name())
        #TODO: Implement this function
    
    def find(self, room: Room, object: Object):
        rospy.loginfo("Finding %s in room: %s", object.get_name(), room.get_name())

        self.go_to_room(room)
        rospy.loginfo("Reached the room...")

        client = actionlib.SimpleActionClient('find', FindAction)
        client.wait_for_server()

        goal = FindGoal()
        goal.room = room.get_name()
        goal.objectType = object.get_name()
        client.send_goal(goal)
        client.wait_for_result()

        


'''----------------------------------------'''

def msg_callback(args, msg):
    action_handler = args['actions']
    rooms = args['rooms']

    action = msg.action
    room = rooms[msg.room]
    obj = Object(msg.object)

    if action == "go_to_room":
        action_handler.go_to_room(room)
    elif action == "photo":
        action_handler.photo(room)
    elif action == "count":
        action_handler.count(room, obj)
    elif action == "find":
        action_handler.find(room, obj)
    else:
        rospy.loginfo("Action not supported")

def main():
    rooms ={
        'Quarto 1': Room('Quarto 1', [-5.995157854315332, 3.0687534759343076]),
        'Quarto 2': Room('Quarto 2', [-2.6291205565668005, 3.936978604700303]),
        'Escritorio': Room('Escritorio', [0.3338533265150007, 3.8938500332742128]),
        'Sanitario1': Room('Sanitario1', [0.6816980018583353, 0.9630639342257676]),
        'Sanitario2': Room('Sanitario2', [1.7447975122639672, -1.4693745387618693]),
        'Sala': Room('Sala', [-1.5000041812678986, -3.999997180836298]),
        'Cozinha': Room('Cozinha', [-3.0660901519164545, -0.8197685726438128]),
        'Vestibulo1': Room('Vestibulo1', [-3.0243864212170455, 1.6197764742613625]),
        'Vestibulo2': Room('Vestibulo2', [-0.44446241721503893, -0.438300400827369]),
        'Everywhere': Room('Everywhere', [0, 0])
    }

    rospy.init_node('message_handler', anonymous=True)

    actions = Actions()


    # Subscribe to the mission topic and set callback function
    callback = rospy.Subscriber("/mission/requested", Request, partial(msg_callback, {'actions': actions, 'rooms': rooms}))



if __name__ == '__main__':
    main()

    while not rospy.is_shutdown():
        rospy.spin()