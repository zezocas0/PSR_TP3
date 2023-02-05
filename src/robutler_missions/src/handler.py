#!/usr/bin/env python3
# Import ROS libraries and messages
from functools import partial
import rospy
from std_msgs.msg import String
from robutler_missions.msg import Request
# from missions import Actions

'''LATER IMPORT THIS FROM missions.py FILE'''
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
        #TODO: Implement this function
    
    def photo(self, room: Room):
        rospy.loginfo("Taking photo in room: %s", room.get_name())
        #TODO: Implement this function

    def count(self, room: Room, object: Object):
        rospy.loginfo("Counting %s in room: %s", object.get_name(), room.get_name())
        #TODO: Implement this function
    
    def find(self, room: Room, object: Object):
        rospy.loginfo("Finding %s in room: %s", object.get_name(), room.get_name())
        #TODO: Implement this function


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
        'Quarto 1': Room('Quarto 1', [0, 0]),
        'Quarto 2': Room('Quarto 2', [0, 0]),
        'Escritorio': Room('Escritorio', [0, 0]),
        'Sanitario1': Room('Sanitario1', [0, 0]),
        'Sanitario2': Room('Sanitario2', [0, 0]),
        'Sala': Room('Sala', [0, 0]),
        'Cozinha': Room('Cozinha', [0, 0]),
        'Vestibulo1': Room('Vestibulo1', [0, 0]),
        'Vestibulo2': Room('Vestibulo2', [0, 0]),
        'Everywhere': Room('Everywhere', [0, 0])
    }

    rospy.init_node('message_handler', anonymous=True)
    rospy.loginfo("Starting camera_listener node")

    actions = Actions()


    # Subscribe to the mission topic and set callback function
    callback = rospy.Subscriber("/mission/requested", Request, partial(msg_callback, {'actions': actions, 'rooms': rooms}))



if __name__ == '__main__':
    main()

    while not rospy.is_shutdown():
        rospy.spin()