#!/usr/bin/env python

# -*- coding: utf-8 -*-

import collections
import sys
import rospy
import numpy as np
from geometry_msgs.msg import PoseArray, PoseStamped
from ackermann_msgs.msg import AckermannDriveStamped
import math
import Utils as utils

# The topic to publish control commands to
PUB_TOPIC = '/vesc/high_level/ackermann_cmd_mux/input/nav_0'

'''
Follows a given plan using constant velocity and PID control of the steering angle
'''
class LineFollower:
  '''
  Initializes the line follower
    plan: A list of length T that represents the path that the robot should follow
          Each element of the list is a 3-element numpy array of the form [x,y,theta]
    pose_topic: The topic that provides the current pose of the robot as a PoseStamped msg
    plan_lookahead: If the robot is currently closest to the i-th pose in the plan,
                    then it should navigate towards the (i+plan_lookahead)-th pose in the plan
    translation_weight: How much the error in translation should be weighted in relation
                        to the error in rotation

    rotation_weight: How much the error in rotation should be weighted in relation
                     to the error in translation
    kp: The proportional PID parameter
    ki: The integral PID parameter
    kd: The derivative PID parameter
    error_buff_length: The length of the buffer that is storing past error values
    speed: The speed at which the robot should travel
  '''
  def __init__(self, plan, pose_topic, plan_lookahead, translation_weight, rotation_weight, kp, ki, kd, error_buff_length, speed):
    # Store the passed parameters
    self.plan = plan
    self.plan_lookahead = plan_lookahead
    # Normalize translation and rotation weights
    self.translation_weight = translation_weight / (translation_weight+rotation_weight)
    self.rotation_weight = rotation_weight / (translation_weight+rotation_weight)
    self.kp = kp
    self.ki = ki
    self.kd = kd
    # The error buff stores the error_buff_length most recent errors and the
    # times at which they were received. That is, each element is of the form
    # [time_stamp (seconds), error]. For more info about the data struct itself, visit
    # https://docs.python.org/2/library/collections.html#collections.deque
    self.error_buff = collections.deque(maxlen=error_buff_length)
    self.speed = speed
    # Creating a publisher to PUB_TOPIC
    self.cmd_pub = rospy.Publisher(PUB_TOPIC, AckermannDriveStamped, queue_size=10)
    # Creating a subscriber to pose_topic, with callback 'self.pose_cb'
    self.pose_sub = rospy.Subscriber(pose_topic, PoseStamped, self.pose_cb)

  '''
  Computes the error based on the current pose of the car
    cur_pose: The current pose of the car, represented as a numpy array [x,y,theta]
 Returns: (False, 0.0) if the end of the plan has been reached. Otherwise, returns
           (True, E) - where E is the computed error
  '''
  def compute_error(self, cur_pose):
    # Find the first element of the plan that is in front of the robot, and remove
    # any elements that are behind the robot. To do this:
    # Loop over the plan (starting at the beginning) For each configuration in the plan
      # If the configuration is behind the robot, remove it from the plan
      #   Will want to perform a coordinate transformation to determine if
      #   the configuration is in front or behind the robot
      # If the configuration is in front of the robot, break out of the loop

    rotation_mx = utils.rotation_matrix(np.pi/2 - cur_pose[2])
    while len(self.plan) > 0:
        x_y_d = [[self.plan[0][0] - cur_pose[0]],[self.plan[0][1] - cur_pose[1]]]
        rot_x_y = np.array(np.matmul(rotation_mx,x_y_d))
        if (self.plan[0][0] - cur_pose[0]) < 0:
            # calculate the distance between the current poase and 
            # the first point in the plan
            first_point = math.sqrt((self.plan[0][0] - cur_pose[0])**2 + (self.plan[0][1] - cur_pose[1])**2) 
	# sqrt((x_p-x_c)^2+(y_p-y_c)^2)
            # calculate distance between current pose and the second 
            # point in plan
            second_point = math.sqrt((self.plan[1][0] - cur_pose[0])**2 + (self.plan[1][1] - cur_pose[1])**2)   
            if second_point < first_point:     # if next is smaller than closest 
                self.plan.pop(0)   # remove the closest plan
            else:
                break
        else:
            if 0 > rot_x_y[1]:

                self.plan.pop(0)
            else:
                break

    # Check if the plan is empty. If so, return (False, 0.0)
        if self.plan == 0:
            return (False, 0.0)

    # At this point, we have removed configurations from the plan that are behind
    # the robot. Therefore, element 0 is the first configuration in the plan that is in
    # front of the robot. To allow the robot to have some amount of 'look ahead',
    # we choose to have the robot head towards the configuration at index 0 + self.plan_lookahead
    # We call this index the goal_index
    goal_idx = min(0+self.plan_lookahead, len(self.plan)-1)

    # Compute the translation error between the robot and the configuration at goal_idx in the plan
    delta = np.array([[self.plan[int(goal_idx)][0] - cur_pose[0]],[self.plan[int(goal_idx)][1] - cur_pose[1]]])

    translation_error = -np.array(np.matmul(rotation_mx, delta))[0]

    # Compute the total error
    # Translation error was computed above
    # Rotation error is the difference in yaw between the robot and goal configuration
    # Be carefult about the sign of the rotation error   
    goal_theta = self.plan[int(goal_idx)][2]

    # if the goal theta and the current theta have opposite signs,
    # we must calculate the distance between pi and the absolute 
    # value of the thetas values and add them together
    # if they have the same sign, we simply subtract current 
    # theta from goal theta
    if goal_theta < 0 and cur_pose[2] > 0:
        num = np.pi - abs(goal_theta)
        num2 = np.pi - cur_pose[2]
        rotation_error = (num + num2)
    elif goal_theta > 0 and cur_pose[2] < 0:
        num = np.pi - abs(cur_pose[2])
        num2 = np.pi - goal_theta
        rotation_error = -1 * (num + num2)
    else:
        rotation_error = goal_theta - cur_pose[2]
    
    # calculate the error
    error = (self.translation_weight * translation_error + self.rotation_weight * rotation_error)
    return True, error

  '''
  Uses a PID control policy to generate a steering angle from the passed error
    error: The current error
  Returns: The steering angle that should be executed
  '''

  def compute_steering_angle(self, error):
    now = rospy.Time.now().to_sec() # Get the current time
    # Compute the derivative error using the passed error, the current time,
    # the most recent error stored in self.error_buff, and the most recent time
    # stored in self.error_buff
    # Add the current error to the buffer
    self.error_buff.append((error, now))
    d_error = error - self.error_buff[0][0]

    # Compute the integral error by applying rectangular integration to the elements
    # of self.error_buff: https://chemicalstatistician.wordpress.com/2014/01/20/rectangular-integration-a-k-a-the-midpoint-rule/
    i_error = 0.5 * (error**2)
    # Compute the steering angle as the sum of the pid errors
    steering_angle = self.kp*error + self.ki*i_error + self.kd*d_error
    return steering_angle

  '''
  Callback for the current pose of the car
    msg: A PoseStamped representing the current pose of the car
    This is the exact callback that we used in our solution, 
    but feel free to change it
  '''

  def pose_cb(self, msg):
    cur_pose = np.array([msg.pose.position.x,
                         msg.pose.position.y,
                         utils.quaternion_to_angle(msg.pose.orientation)])
    success, error = self.compute_error(cur_pose)
    if not success:
      # We have reached our goal
      self.pose_sub = None # Kill the subscriber
      self.speed = 0.0 # Set speed to zero so car stops    
    delta = self.compute_steering_angle(error)
    # Setup the control message
    ads = AckermannDriveStamped()
    ads.header.frame_id = '/map'
    ads.header.stamp = rospy.Time.now()
    ads.drive.steering_angle = delta
    ads.drive.speed = self.speed  
    # Send the control message
    self.cmd_pub.publish(ads)

def main():
  rospy.init_node('line_follower', anonymous=True) # Initialize the node
  # Load these parameters from launch file
  # We provide suggested starting values of params, but you should
  # tune them to get the best performance for your system
  # Look at constructor of LineFollower class for description of each var
  # 'Default' values are ones that probably don't need to be changed (but you could for fun)
  # 'Starting' values are ones you should consider tuning for your system
  # Default val: '/planner_node/car_plan'
  plan_topic = '/planner_node/car_plan'

  # Default val: '/sim_car_pose/pose'
  pose_topic = '/pf/viz/inferred_pose'
  plan_lookahead = rospy.get_param("/plan_lookahead_value") # Starting val: 5
  translation_weight = rospy.get_param("/translation_weight_value") # Starting val: 1.0
  rotation_weight = rospy.get_param("/rotation_weight_value") # Starting val: 0.0
  kp = rospy.get_param("/pid_kp") # Startinig val: 1.0
  ki = rospy.get_param("/pid_ki") # Starting val: 0.0
  kd = rospy.get_param("/pid_kd") # Starting val: 0.0
  error_buff_length = rospy.get_param("/error_buff_length_value") # Starting val: 10
  speed = rospy.get_param("/speed_value") # Default val: 1.0
  raw_input("Press Enter to when plan available...")  # Waits for ENTER key press

  # Use rospy.wait_for_message to get the plan msg
  # Convert the plan msg to a list of 3-element numpy arrays
  #     Each array is of the form [x,y,theta]
  # Create a LineFollower object
  plan_msg = rospy.wait_for_message (plan_topic, PoseArray)
  plan = []
  for msgs in plan_msg.poses:
      plan_poses = np.array([msgs.position.x, msgs.position.y, utils.quaternion_to_angle(msgs.orientation)])
      plan.append(plan_poses)
  pid = LineFollower(plan, pose_topic, plan_lookahead, translation_weight, rotation_weight, kp, ki, kd, error_buff_length, speed)
  rospy.spin() # Prevents node from shutting down

if __name__ == '__main__':
  main()
