#!/usr/bin/env python

import csv
import rospy
import numpy as np
from final.srv import *
import Utils
from nav_msgs.srv import GetMap

# Make sure these paths are correct
START_POINT = '/home/car-user/racecar_ws/src/final/src/start.csv'
TARGET_POINTS = '/home/car-user/racecar_ws/src/final/src/good_waypoints.csv'

PLANNER_SERVICE_TOPIC = 'planner_node/get_car_plan'

if __name__ == '__main__':

    rospy.init_node('planner_test', anonymous=True)

    map_service_name = rospy.get_param("~static_map", "static_map")
    print("Getting map from service: ", map_service_name)
    rospy.wait_for_service(map_service_name)
    map_info = rospy.ServiceProxy(map_service_name, GetMap)().map.info

    rospy.wait_for_service(PLANNER_SERVICE_TOPIC)
    get_plan = rospy.ServiceProxy(PLANNER_SERVICE_TOPIC, GetPlan)

    count = 0
    big_plan = np.array([])
    big_plan_2 = []
    start_pose = []	

    # open file csv file that contains start point
    with open(START_POINT) as csvfile: 
        startreader = csv.reader(csvfile, delimiter=' ', quotechar='|')
	for row in startreader:
            start = row[0]
            # ignore first row that's (x,y)
	    for i in range(len(start)):
	        if start[i] == 'x':
                    break  
		elif start[i] == ",":
                    # value before comma is x
		    start_x = start[0:i]
                    # value after comma is y
		    start_y = start[i + 1:len(start)]
		    start_pose = [float(start_x), float(start_y), 0.0]
    # open csv file containing target points
    with open(TARGET_POINTS) as csvfile:
        spamreader = csv.reader(csvfile, delimiter=' ', quotechar='|')
        for row in spamreader:
	    target = row[0]
            for i in range(len(target)):
		if target[i] == 'x':
                    break
		elif target[i] == ",":
                    # if this is the beginning of the plan, the first 
                    # point should be the start point. Otherwise it 
                    # should be the previous target point
		    if count == 0:
		        old_target = start_pose
		    else:
			old_target = new_target
		    count += 1
		    target_x = target[0:i]
		    target_y = target[i + 1:len(target)]
		    new_target = [float(target_x), float(target_y), 0.0]  
		    try:
		        resp = get_plan(Utils.map_to_world(old_target, map_info), Utils.map_to_world(new_target, map_info))
			big_plan = np.append((big_plan,np.array(resp.plan).reshape(-1, 3)),axis=0)
			big_plan_2.append(big_plan,np.array(resp.plan).reshape(-1, 3))
			print('bigplan2',big_plan_2)
			print big_plan
			print resp.success
		    except rospy.ServiceException, e:
		        print 'Service call failed: %s' % e

