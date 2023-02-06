#!/usr/bin/env python3
import json
import rospy
import std_msgs.msg
from robutler_missions.msg import Actions as Actions_msg, Objects as Objects_msg, Rooms as Rooms_msg, Properties as Properties_msg

properties = Properties_msg()

objects = rospy.get_param("/objects")
rooms = rospy.get_param("/rooms")

for i in objects:
    obj = Objects_msg()
    obj.name = i
    properties.objects.append(obj)

for i in rooms:
    rm = Rooms_msg()
    rm.name = i['name']
    rm.x = i['coordinates'][0]
    rm.y = i['coordinates'][1]
    properties.rooms.append(rm)

go_to_room = Actions_msg()
go_to_room.action = 'go_to_room'
go_to_room.only_rooms = True
go_to_room.only_objects = False


photo = Actions_msg()
photo.action = 'photo'
photo.only_rooms = False
photo.only_objects = False

count = Actions_msg()
count.action = 'count'
count.only_rooms = False
count.only_objects = True

find = Actions_msg()
find.action = 'find'
find.only_rooms = False
find.only_objects = False

properties.actions = [go_to_room, photo, count, find]

rospy.init_node('missions', anonymous=True)
pub = rospy.Publisher('/mission', Properties_msg, queue_size=10)
rate = rospy.Rate(10) # 10hz


while not rospy.is_shutdown():
    pub.publish(properties)
    rate.sleep()