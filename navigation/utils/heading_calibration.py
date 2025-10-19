import rospy 
import rospkg 
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from sensor_msgs.msg import NavSatFix
import time
import math
import pymap3d as pm
import matplotlib.pyplot as plt

def center_data(data, exclude_indices=[]):
    reduced_data = np.delete(data, exclude_indices, axis=0)
    center = np.array([reduced_data.mean(axis=0)])
    return center, data - center


def compute_cross_covariance(P, Q, correspondences, kernel=lambda diff: 1.0): 
    cov = np.zeros((2, 2))
    exclude_indices = []
    for i, j in correspondences:
        p_point = P.T[:, [i]]
        q_point = Q.T[:, [j]]
        weight = kernel(p_point - q_point)
        if weight < 0.01: exclude_indices.append(i)
        cov += weight * q_point.dot(p_point.T)
    return cov, exclude_indices

def encoderCallback(msg): 
    global vel_x 
    vel_x = msg.twist.twist.linear.x 
    

def gpsCallback(msg):
    global local_pos, gps_path, local_path, gps_datum, local_datum, theta_list, vel_x, calibration_flag
    if gps_datum == [] and local_datum == [] and calibration_flag == False:
        gps_datum = np.asarray([msg.latitude, msg.longitude, msg.altitude])
        local_datum = np.asarray(local_pos)
    elif np.shape(gps_path)[0] < 150 and vel_x > 0 and calibration_flag == False:
        gps_pos = pm.geodetic2enu(msg.latitude, msg.longitude, msg.altitude, gps_datum[0], gps_datum[1], gps_datum[2])  
        local_pos -= local_datum
        gps_path.append(gps_pos)
        local_path.append(local_pos)
    elif np.shape(gps_path)[0] == 150 and calibration_flag == False: 
        P = np.asarray(gps_path)[:,:-1]
        Q = np.asarray(local_path)[:,:-1]

        center_of_P, P_centered = center_data(P)
        center_of_Q, Q_centered = center_data(Q)
        correspondences = []
        for i in range(P.shape[0]):
            correspondences.append((i, i))
        cov, _ = compute_cross_covariance(P_centered, Q_centered, correspondences)
        U, S, V_T = np.linalg.svd(cov)
        R = U.dot(V_T)
	print 'Is R in SO(3)?: ', np.linalg.det(R)>0

    	P_correct = R.dot(P_centered.T).T + center_of_Q
        theta = np.arccos((np.trace(R)) / 2.0)
	theta_list.append(theta)
	rospy.set_param("/calibrated_yaw_offset", str(R[0,0]) + ' ' + str(R[0,1]) + ' ' + str(R[1,0]) + ' ' + str(R[1,1]))
        print 'Rotation angle (deg): ', theta*180.0/np.pi
        gps_path, local_path, gps_datum, local_datum = [], [], [], []
    	plt.figure()

    	plt.plot(P[:,0], P[:,1], 'r.', label='GPS')
    	plt.plot(Q[:,0], Q[:,1], 'b1', label='LIO')
    	plt.plot(P_correct[:,0], P_correct[:,1], 'g+', label='GPS corrected')
    	plt.legend()
    	plt.savefig('online_calibration_results.png', dpi=600, bbox_inches='tight')
    
def locCallback(msg):
    global local_pos
    local_pos = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]


if __name__ == "__main__":
    gps_path, local_path, local_pos, gps_datum, local_datum, theta_list, vel_x, calibration_flag = [], [], [], [], [], [], [], False
    print "Calibrating IMU..."
    rospy.init_node("heading_calibration")
    gps_sub = rospy.Subscriber("ublox_position_receiver/fix", NavSatFix, gpsCallback)
    loc_sub = rospy.Subscriber("map_base_link/odometry", Odometry, locCallback)
    vel_sub = rospy.Subscriber("odom", Odometry, encoderCallback)
    rospy.spin()


