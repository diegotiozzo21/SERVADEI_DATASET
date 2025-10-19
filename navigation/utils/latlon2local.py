#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu, NavSatFix
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_matrix
import pymap3d as pm
import numpy as np
import math
import tf
import time
import os
import matplotlib.pyplot as plt

mag_decl = np.fromstring(rospy.get_param("/calibrated_yaw_offset"), dtype=float, sep=' ').reshape(2,2)


lat0, lon0, h0 = [0]*3
sub_imu = []

def plot_waypoints(waypoints,t_map_robot,q_map_robot):
    plt.figure()
    T_map_robot = quaternion_matrix(q_map_robot)
    v = np.array([t_map_robot[0], t_map_robot[1], t_map_robot[2]])+np.array([T_map_robot[0,0], T_map_robot[1,0], T_map_robot[2,0]])
    plt.plot([t_map_robot[0], v[0]], [t_map_robot[1], v[1]], 'r-')
    v = np.array([t_map_robot[0], t_map_robot[1], t_map_robot[2]])+np.array([T_map_robot[0,1], T_map_robot[1,1], T_map_robot[2,1]])
    plt.plot([t_map_robot[0], v[0]], [t_map_robot[1], v[1]], 'g-')
    for waypoint in waypoints:
        plt.scatter(waypoint[0], waypoint[1])
    plt.xlabel('E (m)')  
    plt.ylabel('N (m)')
    plt.title('Waypoints in the map frame')
    plt.axis('equal')
    plt.savefig('waypoints.png')


def get_datum(msg):
    global count, lat0, lon0, h0, sub_imu, sub_gps, t_map_robot, q_map_robot, mag_decl
    lat0 = msg.latitude
    lon0 = msg.longitude
    h0 = msg.altitude
    sub_gps.unregister()
        
    waypoints = []
    with open('collected_waypoints.txt', 'r') as fr: 
        for line in fr.readlines():
            l = line.strip().split(' ')
            wgs84_goal = [float(l[0]), float(l[1]), float(l[2]), float(l[3])]   
            base_goal = pm.geodetic2enu(wgs84_goal[0], wgs84_goal[1], wgs84_goal[2], lat0, lon0, h0,  ell = pm.Ellipsoid('wgs84'))
            R_mag_decl = np.array([ [mag_decl[0,0], mag_decl[0,1], 0], [mag_decl[1,0], mag_decl[1,1], 0], [0,0,1] ])
	    base_goal_calib = np.dot(R_mag_decl, np.array([base_goal[0], base_goal[1], base_goal[2]]).reshape(3,1))
            waypoint = [float(base_goal_calib[0] + t_map_robot[0]), float(base_goal_calib[1] + t_map_robot[1]), float(base_goal_calib[2] + t_map_robot[2]), float(wgs84_goal[3])]
            waypoints.append(waypoint)
    fr.close()

    # Write points to a txt file
    if os.path.exists("trans_points.txt"):
      os.remove("trans_points.txt")    
    f = open("trans_points.txt", "w")
    for waypoint in waypoints: 
        f.writelines(str(waypoint[0]) + ' ' + str(waypoint[1]) + ' ' + str(waypoint[2]) + ' ' + str(waypoint[3]) + '\n')
    f.close()
    plot_waypoints(waypoints,t_map_robot,q_map_robot)
    rospy.loginfo("Waypoints stored in the txt file: trans_points.txt")
               
rospy.init_node('latlon2local')
sub_gps = rospy.Subscriber ('/ublox_position_receiver/fix', NavSatFix, get_datum)
trans_listener = tf.TransformListener()

r = rospy.Rate(1)
while not rospy.is_shutdown(): 
    try:
        (t_map_robot, q_map_robot) = trans_listener.lookupTransform( 'map', 'base_link', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        continue    
    r.sleep()












