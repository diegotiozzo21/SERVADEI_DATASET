#!/usr/bin/env python 
import rospy
from sensor_msgs.msg import Imu, NavSatFix
from tf.transformations import euler_from_quaternion
import numpy as np
import math
import time 
import os 
import sys

def imu_callback(data):
    global YAW, imu_sub
    q = data.orientation
    roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
    YAW = yaw*180/math.pi 
    imu_sub.unregister()
    
def ublox_callback(ublox_data):
    global LAT, LON, H, imu_sub, f
    imu_sub = rospy.Subscriber("/imu/data", Imu, imu_callback, queue_size=1)
    LAT = ublox_data.latitude
    LON = ublox_data.longitude
    H = ublox_data.altitude
    
def listener():
    global LAT, LON, H, f
    while not rospy.is_shutdown():
      ublox_sub = rospy.Subscriber("/ublox_position_receiver/fix", NavSatFix, ublox_callback, queue_size=1)
      char = sys.stdin.read(1) 
      if ord(char) == 32:          
          print("Collected waypoint: ")
          print("Lat: " + str(LAT) + "\n" + "Lon: " + str(LON) + "\n" + "H: " + str(H) + "\n" + "Yaw: " + str(YAW))
          f.writelines(str(LAT) + " " + str(LON) + " " + str(H) + " " + str(YAW) + "\n")
          ublox_sub.unregister()       
    

if __name__ == '__main__':
    rospy.init_node('gps_collection', anonymous=True)
    LAT, LON, H, YAW, char = 0, 0, 0, 0, 0
    if os.path.exists("collected_waypoints.txt"):
        os.remove("collected_waypoints.txt")
    f = open("collected_waypoints.txt", "w")
    listener()   
    f.close()


