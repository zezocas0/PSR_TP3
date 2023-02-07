#!/usr/bin/env python3

import random
import sys

import rospy
import rospkg
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose, Point, Quaternion

rospy.init_node('insert_object',log_level=rospy.INFO)

# get an instance of RosPack with the default search paths
rospack = rospkg.RosPack()
package_path = rospack.get_path('psr_apartment_description') + '/description/models/'


model_names = ['blue_cube', 'green_cube', 'red_cube']

places = ["In the bed of the large bedroom", "In the living room shelf", "In the small room furniture", "In the kitchen cabinet"]

places_pose = {place: None for place in places}
places_pose['In the bed of the large bedroom'] = [{'pose':Pose(position=Point(x=-5.69, y=4.37, z=1), orientation=Quaternion(x=0,y=0,z=0,w=1)),'room':'large_bedroom', 'place': 'bed'}]
places_pose['In the living room shelf'] = [{'pose':Pose(position=Point(x=-2.169804, y=-2.422212, z=0.115565), orientation=Quaternion(x=0,y=0,z=0,w=1)),'room':'sala', 'place': 'shelf'},
    {'pose':Pose(position=Point(x=-1.530873, y=-2.383779, z=0.115565), orientation=Quaternion(x=0,y=0,z=0,w=1)),'room':'sala', 'place': 'shelf'},
    {'pose':Pose(position=Point(x=-3.401368, y=-2.338209, z=0.495562), orientation=Quaternion(x=0,y=0,z=0,w=1)),'room':'sala', 'place': 'shelf'},
    {'pose':Pose(position=Point(x=-2.694710, y=-2.400191, z=0.865564), orientation=Quaternion(x=0,y=0,z=0,w=1)),'room':'sala', 'place': 'shelf'},
    {'pose':Pose(position=Point(x=-3.468925, y=-2.386329, z=1.255566), orientation=Quaternion(x=0,y=0,z=0,w=1)),'room':'sala', 'place': 'shelf'},
    {'pose':Pose(position=Point(x=-1.761279, y=-2.208371, z=1.255566), orientation=Quaternion(x=0,y=0,z=0,w=1)),'room':'sala', 'place': 'shelf'}]
places_pose['In the small room furniture'] = [{'pose':Pose(position=Point(x=-1.861356, y=3.959174, z=0.5), orientation=Quaternion(x=0,y=0,z=0,w=1)),'room':'small_room', 'place': 'furniture'}]
places_pose['In the kitchen cabinet'] = [{'pose':Pose(position=Point(x=-3.172834, y=-1.785831, z=0.956774), orientation=Quaternion(x=0,y=0,z=0,w=1)),'room':'kitchen', 'place': 'cabinet'},
    {'pose':Pose(position=Point(x=-1.443919, y=-0.811314, z=0.956774), orientation=Quaternion(x=0,y=0,z=0,w=1)),'room':'kitchen', 'place': 'cabinet'},]


count = 0
while True:
    if len(places) == 0:
        print("There is nothing more you can do")
        sys.exit(0)
    print("\n\nWhere do you want to place some colores cubes?\n")
    for i in range(len(places)):
        print(str(i+1) + ") " + places[i])
    print("0) to exit\n")

    place_idx = int(input("Enter the number of the place: "))

    if place_idx == 0:
        print("Exiting...")
        sys.exit(0)
    
    if place_idx < 0 or place_idx > len(places):
        print("Invalid place number")
        continue

    place = places.pop(place_idx-1)
    poses = places_pose[place]

    for pose in poses:
        model_name = random.choice(model_names)

        f = open( package_path + model_name + '/model.sdf' ,'r')
        sdff = f.read()

        rospy.wait_for_service('gazebo/spawn_sdf_model')
        spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)

        model_placement = pose
        name = model_name + '_in_' + model_placement['place'] + '_of_' + model_placement['room'] + '_' + str(count)
        spawn_model_prox(name, sdff, model_name, model_placement['pose'], "world")
        print("Placed " + name)
        count += 1