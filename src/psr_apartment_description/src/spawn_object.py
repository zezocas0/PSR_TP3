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


#Add here mode poses
placements = []
placements.append({'pose':Pose(position=Point(x=-5.69, y=4.37, z=0.6), orientation=Quaternion(x=0,y=0,z=0,w=1)),
              'room':'large_bedroom', 'place': 'bed'})
placements.append({'pose':Pose(position=Point(x=-7.33, y=5.29, z=0.58), orientation=Quaternion(x=0,y=0,z=0,w=1)),
              'room':'large_bedroom', 'place': 'bedside_cabinet'})


model_names = ['blue_cube', 'green_cube', 'red_cube']

places = ["In the bed of the large bedroom", "In the living room shelf"]

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
    
    if place_idx < 0 or place_idx > len(placements):
        print("Invalid place number")
        continue

    place = places.pop(place_idx-1)

    print(place)
    continue


    # Add here several models. All should be added to the robutler_description package
    model_name = random.choice(model_names)

    f = open( package_path + model_name + '/model.sdf' ,'r')
    sdff = f.read()

    rospy.wait_for_service('gazebo/spawn_sdf_model')
    spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)


    model_placement = random.choice(placements)
    name = model_name + '_in_' + model_placement['place'] + '_of_' + model_placement['room']
    spawn_model_prox(name, sdff, model_name, model_placement['pose'], "world")