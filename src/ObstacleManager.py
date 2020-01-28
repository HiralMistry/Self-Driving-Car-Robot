#!/usr/bin/env python

import csv
import cv2
import math
import numpy
import Utils
import rospy 
import numpy as np

from matplotlib import pyplot as plt

from nav_msgs.srv import GetMap
#from lab3.srv import *

# Make sure path is correct
BAD_WAYPOINTS = '/home/car-user/racecar_ws/src/final/src/bad_waypoints.csv'

class ObstacleManager(object):

	def __init__(self, mapMsg, car_width, car_length, collision_delta):
		
		self.map_info = mapMsg.info
		self.mapImageGS = numpy.array(mapMsg.data, dtype=numpy.uint8).reshape(
			(mapMsg.info.height, mapMsg.info.width, 1))
		#print (self.mapImageGS," self.mapImageGS\n")

		# Retrieve the map dimensions
		height, width, channels = self.mapImageGS.shape
		self.mapHeight = height
		self.mapWidth = width
		self.mapChannels = channels

		# Binarize the Image
		self.mapImageBW = 255 * numpy.ones_like(self.mapImageGS, dtype=numpy.uint8)
		self.mapImageBW[self.mapImageGS == 0] = 0 #0 is permissible, 255 not permissible
		#print self.mapImageBW[0]

		# Obtain the car length and width in pixels
		self.robotWidth = int(car_width / self.map_info.resolution + 0.5)
		self.robotLength = int(car_length / self.map_info.resolution + 0.5)
		self.collision_delta = collision_delta
		box_length = np.sqrt(self.robotLength**2 + self.robotWidth**2)
		self.box = np.array([int(box_length),int(box_length)])
		# open csv file with bad waypoints
		with open(BAD_WAYPOINTS) as csvfile:
    		    spamreader = csv.reader(csvfile, delimiter=' ', quotechar='|')
		    # read in bad waypoints and set a box around waypoints
                    # to be non permissible
	            for row in spamreader:
				waypoint = row[0]
				num = 0
				for i in range(len(waypoint)):
				    if waypoint[i] == 'x':
				        break
					# get waypoints that will be used 
                                        # as indeces
				    elif waypoint[i] == ",":
				        bad_x = int(waypoint[0:i])
				        bad_y = int(waypoint[i + 1:len(waypoint)])
					bad_y = int(self.mapHeight - bad_y)
					
					# set box around waypoint 
                                        # to be non permissble 
					self.mapImageBW[bad_y - (self.box[0] / 2):(self.box[0] / 2) + bad_y,bad_x - (self.box[1] / 2):(self.box[1] / 2) + bad_x,0] = 255

	# Check if the passed config is in collision
	# config: The configuration to check (in meters and radians)
	# Returns False if in collision, True if not in collision
	def get_state_validity(self, config):
		# Convert the configuration to map-coordinates -> mapConfig is in pixel-space
		mapConfig = Utils.world_to_map(config, self.map_info)
		# Convert the configuration to map-coordinates -> mapConfig is in pixel-space
		# Check box of map coordinates to see if car would 
                # be in collision
		box_check = self.mapImageBW[mapConfig[1] - (self.box[0] / 2):(self.box[0] / 2) + mapConfig[1],mapConfig[0] - (self.box[1] / 2):(self.box[1] / 2) + mapConfig[0],0]

		if np.sum(box_check) > 0:
			#print("false")			
			return False
		
		
		return True

	# Discretize the path into N configurations, where N = path_length / self.collision_delta
	#
	# input: an edge represented by the start and end configurations
	#
	# return three variables:
	# list_x - a list of x values of all intermediate points in the path
	# list_y - a list of y values of all intermediate points in the path
	# edgeLength - The euclidean distance between config1 and config2
	def discretize_edge(self, config1, config2):
		list_x, list_y = [], []
                # get the length of the edge
		edgeLength = np.sqrt((abs(config1[0] - config2[0])**2) + (abs(config1[1] - config2[1])**2))
                # if the length of the edge is zero (config1=congif2)
                # return just that point
		if(edgeLength==0 or np.array_equal(config1, config2)):
			list_x.append(config1[0])
			list_y.append(config1[1])
			return list_x, list_y, 0
                # number of path points between config1 and config
		N = edgeLength / self.collision_delta
                # use y = mx + b to get path
                # increment x from config1 x value to config 2 x 
                # value and calculate y
		x_incr = abs(config2[0]-config1[0]) / N
		x = config1[0]		
		slope = (float(config2[1] - config1[1]) / float(config2[0] - config1[0]))		
		b = config1[1] - slope * config1[0]
		#print('N: ',N, 'slope: ',slope,'b: ', b,'x_incr', x_incr)		
                # add config1 to the path list
		list_x.append(config1[0])
		list_y.append(config1[1])
                # calculate path points and append to path list
		for i in range(int(N)):
			# increment x
			if (config1[0] < config2[0]):
				x = x + x_incr
			if (config1[0] > config2[0]):
				x = x - x_incr
                        # calculate y
			y = slope * x + b
			# append x and y to path lists
			list_x.append(x)
			list_y.append(y)
                # add config2 go path list
		list_x.append(config2[0])
		list_y.append(config2[1])
		#plot path		
		#plt.plot(list_x,list_y,'oc')
		#plt.show()	
		return list_x, list_y, edgeLength


	# Check if there is an unobstructed edge between the passed configs
	# config1, config2: The configurations to check (in meters and radians)
	# Returns false if obstructed edge, True otherwise
	def get_edge_validity(self, config1, config2):
                # make sure config1 and config2 are not in collision
		if not self.get_state_validity(config1) or not self.get_state_validity(config2):
			return False
                # call discretize_edge function to get path
		list_x, list_y, edgeLength = self.discretize_edge(config1, config2)
		# call get_state_validity function for all points in 
                # the path 
		for i in range(len(list_x)):
			# return false if point is in collision
			if not self.get_state_validity(np.array((list_x[i], list_y[i]))):
				return False
		
		# no collision between config1 and 2
		return True
	#get_state_validity(ObstacleManager(), np.array([0,0]))
# Write Your Test Code For Debugging
if __name__ == '__main__':
	# Write test code here!
	print("main")
	rospy.init_node('obstacle_manager', anonymous=True)
	car_width = rospy.get_param("/car_kinematics/car_width", 0.33)
 	car_length = rospy.get_param("/car_kinematics/car_length", 0.33)
	car_width = 0.33
	car_length = 0.33
	collision_delta = rospy.get_param("~collision_delta", 0.05)
	map_service_name = rospy.get_param("~static_map", "static_map")
	print("map wait")
  	rospy.wait_for_service(map_service_name)
  	map_msg = rospy.ServiceProxy(map_service_name, GetMap)().map
	om = ObstacleManager(map_msg, car_width, car_length, collision_delta)
	om.get_state_validity(np.array([1910,340])) 
	#om.discretize_edge(np.array([-2,-10]),np.array([10,-50]))
	#om.get_edge_validity(np.array([15,12.3]),np.array([0,0]))
