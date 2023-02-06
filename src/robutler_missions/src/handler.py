#!/usr/bin/env python3
# Import ROS libraries and messages
from functools import partial
import rospy
import actionlib

from datetime import datetime
from std_msgs.msg import String
from robutler_missions.msg import Request
from robutler_controller.msg import MoveRobutlerAction, MoveRobutlerGoal, TakePhotoAction, TakePhotoGoal, FindAction, FindGoal, CountAction, CountGoal
import geometry_msgs.msg

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

    def get_closest_room(self):
        order = ["Sala",  "Sanitario2", "Vestibulo2", "Sanitario1", "Escritorio",  "Quarto 2",  "Vestibulo1", "Quarto 1", "Cozinha"]
        #get robot pose
        pose = rospy.wait_for_message('/amcl_pose', geometry_msgs.msg.PoseWithCovarianceStamped, timeout=0.1)
        x = pose.pose.pose.position.x
        y = pose.pose.pose.position.y
        
        #get closest room
        closest_room = None
        for room in self.rooms:
            room = self.rooms[room]
            coordinates = room.coordinates
            distance = ((coordinates[0] - x)**2 + (coordinates[1] - y)**2)**0.5
            if closest_room is None or distance < closest_room[1]:
                closest_room = (room, distance)
        
        # find index of closest room
        index = order.index(closest_room[0].get_name())
        # rotate order
        new_order = order[index:] + order[:index]
        return new_order

    def go_to_room(self, room: Room):
        rospy.loginfo("Going to %s", room.get_name())

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
        rospy.loginfo("Counting %s in %s", object.get_name(), room.get_name())

        rooms = [room]

        if room.get_name() == "Everywhere":
            order = self.get_closest_room()
            rooms = [self.rooms[room_name] for room_name in order]

        for room in rooms:
            self.go_to_room(room)
            rospy.loginfo(f"Reached {room.get_name()}...")

            client = actionlib.SimpleActionClient('count', CountAction)
            client.wait_for_server()

            goal = CountGoal()
            goal.room = room.get_name()
            goal.objectType = object.get_name()
            goal.color = "red" #TODO: Get Color Somehow
            client.send_goal(goal)
            client.wait_for_result()
    


    def find(self, room: Room, object: Object):
        rospy.loginfo("Finding %s in %s", object.get_name(), room.get_name())

        rooms = [room]

        if room.get_name() == "Everywhere":
            order = self.get_closest_room()
            rooms = [self.rooms[room_name] for room_name in order]

        for room in rooms:
            self.go_to_room(room)
            rospy.loginfo(f"Reached {room.get_name()}...")
            client = actionlib.SimpleActionClient('find', FindAction)
            client.wait_for_server()

            goal = FindGoal()
            goal.room = room.get_name()
            goal.objectType = object.get_name()
            client.send_goal(goal)
            client.wait_for_result()

            #TODO: Stop searching when finding the object


'''----------------------------------------'''

def msg_callback(args, msg):

    action_handler = args['actions']
    rooms = args['rooms']


    action = msg.action
    if "specific location" in msg.room:
        room = Room("Specific Location", [float(msg.room.split(":")[1]), float(msg.room.split(":")[2])])
    else:
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
    param_rooms = rospy.get_param("/rooms")
    rooms = {}
    for i in param_rooms:
        rooms[i['name']] = Room(i['name'], i['coordinates'])

    rospy.init_node('message_handler', anonymous=True)

    actions = Actions()
    actions.rooms = rooms


    # Subscribe to the mission topic and set callback function
    rospy.Subscriber("/mission/requested", Request, partial(msg_callback, {'actions': actions, 'rooms': rooms}))



if __name__ == '__main__':
    main()

    while not rospy.is_shutdown():
        rospy.spin()