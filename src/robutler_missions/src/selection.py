#!/usr/bin/env python3

import rospy
import std_msgs.msg
import robutler_missions.msg 
import visualization_msgs.msg

# from interactive_markers.msg import InteractiveMarkerUpdate
#wait for a single message

def get_menu_feedback(data):
    rospy.loginfo(menu_entries_processed[data.menu_entry_id])
    #generate unique request id
    uid = str(rospy.get_rostime())
    pub.publish(robutler_missions.msg.Request(action=menu_entries_processed[data.menu_entry_id]["action"], room=menu_entries_processed[data.menu_entry_id]["room"], object=menu_entries_processed[data.menu_entry_id]["object"], request_id=uid))

rospy.init_node("menu", anonymous=True)
pub = rospy.Publisher("/mission/requested", robutler_missions.msg.Request, queue_size=10)
rospy.Subscriber("/menu/feedback", visualization_msgs.msg.InteractiveMarkerFeedback, get_menu_feedback)
menu = rospy.wait_for_message('/menu/update_full', visualization_msgs.msg.InteractiveMarkerInit)
menu_entries = menu.markers[0].menu_entries

menu_entries_processed = {}

actions = [x for x in menu_entries if x.parent_id == 0]

rooms = []
for i in menu_entries:
    for j in actions:
        if i.parent_id == j.id:
            rooms.append(i)
            break

objects = []
for i in menu_entries:
    for j in rooms:
        if i.parent_id == j.id:
            objects.append(i)
            break

print(rooms)

for i in objects:
    _object = i
    for i in rooms:
        if i.id == _object.parent_id:
            _room = i
            for i in actions:
                if i.id == _room.parent_id:
                    _action = i
                    menu_entries_processed[_object.id]={"action": _action.title, "room": _room.title, "object": _object.title}
                    break
#remove all rooms from rooms that are in the menu_entries_processed
for i in rooms:
    if i.title in menu_entries_processed:
        rooms.remove(i)
for room in rooms:
    for action in actions:
        if action.id == room.parent_id:
            menu_entries_processed[room.id]={"action": action.title, "room": room.title, "object": "Room"}
            break
        

rospy.spin()