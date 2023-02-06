#!/usr/bin/env python3

from __future__ import print_function

import rospy

from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
import robutler_missions.msg
server = None
marker_pos = 0
rooms = []
objects = []
actions = []

menu_handler = MenuHandler()

h_first_entry = 0
h_mode_last = 0

def enableCb( feedback ):
    print(feedback)
    handle = feedback.menu_entry_id
    state = menu_handler.getCheckState( handle )


    menu_handler.reApply( server )
    rospy.loginfo(handle)
    rospy.loginfo(state)
    server.applyChanges()


def makeBox( msg ):
    marker = Marker()

    marker.type = Marker.CUBE
    marker.scale.x = msg.scale * 0.45
    marker.scale.y = msg.scale * 0.45
    marker.scale.z = msg.scale * 0.45
    marker.color.r = 0.5
    marker.color.g = 0.5
    marker.color.b = 0.5
    marker.color.a = 1.0

    return marker

def makeBoxControl( msg ):
    control = InteractiveMarkerControl()
    control.always_visible = True
    control.markers.append( makeBox(msg) )
    msg.controls.append( control )
    return control

def makeEmptyMarker( dummyBox=True ):
    global marker_pos
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "base_link"
    int_marker.pose.position.y = -3.0 * marker_pos
    marker_pos += 1
    int_marker.scale = 1
    return int_marker

def makeMenuMarker( name ):
    int_marker = makeEmptyMarker()
    int_marker.name = name

    control = InteractiveMarkerControl()

    control.interaction_mode = InteractiveMarkerControl.BUTTON
    control.always_visible = True

    control.markers.append( makeBox( int_marker ) )
    int_marker.controls.append(control)

    server.insert( int_marker )

def deepCb( feedback ):
    rospy.loginfo("The deep sub-menu has been found.")

def initMenu(properties):

    for action in properties.actions:
        entry = menu_handler.insert( action.action, callback=enableCb )
        for room in properties.rooms:
            if not (action.only_rooms and room.name=="Everywhere"):
                entry2 = menu_handler.insert( room.name, parent=entry, callback=enableCb )
                print(action.action, action.only_rooms)
                if not action.only_rooms:
                    for object in  properties.objects:
                        if not (action.only_objects and object.name=="Room"):
                            entry3 = menu_handler.insert( object.name, parent=entry2, callback=enableCb )


def loadOptions(args: robutler_missions.msg.Properties):
    for i in args.actions:
        actions.append(i.action)
    for i in args.objects:
        objects.append(i.name)
    for i in args.rooms:
        rooms.append(i.name)

if __name__=="__main__":
    rospy.init_node("menu", anonymous=True)
    
    #check if menu is already initialized



    #rospy.Subscriber("/mission", robutler_missions.msg.Properties, loadOptions)
    properties = rospy.wait_for_message("/mission", robutler_missions.msg.Properties, timeout=None)
    
    server = InteractiveMarkerServer("menu")

    initMenu(properties)
    
    makeMenuMarker( "marker1" )
    

    menu_handler.apply( server, "marker1" )
    
    server.applyChanges()

    rospy.spin()
