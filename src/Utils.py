#!/usr/bin/env python

import rospy
import numpy as np

from std_msgs.msg import Header
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Pose, PoseStamped, PoseArray, Quaternion, PolygonStamped, Polygon, Point32, \
    PoseWithCovarianceStamped, PointStamped
import tf.transformations
import tf
import matplotlib.pyplot as plt


def angle_to_quaternion(angle):
    """Convert an angle in radians into a quaternion _message_."""
    return Quaternion(*tf.transformations.quaternion_from_euler(0, 0, angle))


def quaternion_to_angle(q):
    """Convert a quaternion _message_ into an angle in radians.
    The angle represents the yaw.
    This is not just the z component of the quaternion."""
    x, y, z, w = q.x, q.y, q.z, q.w
    roll, pitch, yaw = tf.transformations.euler_from_quaternion((x, y, z, w))
    return yaw


def rotation_matrix(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.matrix([[c, -s], [s, c]])


def world_to_map(pose, map_info):
    # equivalent to map_to_grid(world_to_map(pose))
    # operates in place
    scale = map_info.resolution
    angle = -quaternion_to_angle(map_info.origin.orientation)
    config = [0.0, 0.0]
    # translation
    config[0] = (1.0 / float(scale)) * (pose[0] - map_info.origin.position.x)
    config[1] = (1.0 / float(scale)) * (pose[1] - map_info.origin.position.y)

    # rotation
    c, s = np.cos(angle), np.sin(angle)
    # we need to store the x coordinates since they will be overwritten
    temp = np.copy(config[0])
    config[0] = int(c * config[0] - s * config[1])
    config[1] = int(s * temp + c * config[1])

    return config


def map_to_world(pose, map_info):
    scale = map_info.resolution
    angle = quaternion_to_angle(map_info.origin.orientation)

    # rotate
    config = np.array([pose[0], map_info.height - pose[1], pose[2]])
    # rotation
    c, s = np.cos(angle), np.sin(angle)
    # we need to store the x coordinates since they will be overwritten
    temp = np.copy(config[0])
    config[0] = c * config[0] - s * config[1]
    config[1] = s * temp + c * config[1]

    # scale
    config[:2] *= float(scale)

    # translate
    config[0] += map_info.origin.position.x
    config[1] += map_info.origin.position.y
    config[2] += angle

    return config
'''
  Creates a header with the given frame_id and stamp. Default value of stamp is
  None, which results in a stamp denoting the time at which this function was called
    frame_id: The desired coordinate frame
    stamp: The desired stamp
    Returns: The resulting header
'''
def make_header(frame_id, stamp=None):
    if stamp == None:
        stamp = rospy.Time.now()
    header = Header()
    header.stamp = stamp
    header.frame_id = frame_id
    return header

'''
  Converts a list of particles to a list of pose messages
    particles: A list of particles, where each element is itself a list of the form [x,y,theta]
    Returns: A list of equivalent geometry_msgs/Pose messages
'''
def particles_to_poses(particles):
    return map(particle_to_pose, particles)

'''
  Converts a particle to a pose message
    particle: The particle to convert - [x,y,theta]
    Returns: An equivalent geometry_msgs/Pose
'''
def particle_to_pose(particle):
    pose = Pose()
    pose.position.x = particle[0]
    pose.position.y = particle[1]
    pose.orientation = angle_to_quaternion(particle[2])
    return pose

