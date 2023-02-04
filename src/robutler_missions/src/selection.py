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

for object in menu_entries:
    for i in menu_entries:
        if i.id == object.parent_id:
            room = i
            for i in menu_entries:
                if i.id == room.parent_id:
                    action = i
                    menu_entries_processed[object.id]={"action": action.title, "room": room.title, "object": object.title}
                    break

rospy.spin()