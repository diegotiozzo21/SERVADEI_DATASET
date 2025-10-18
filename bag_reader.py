from pathlib import Path
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore
import numpy as np
import matplotlib.pyplot as plt 
import os


class ReadBag:

    def __init__(self):
        self.bagpath = Path('./2024-09-22-15-03-28.bag')
        self.typestore = get_typestore(Stores.ROS1_NOETIC)  

    def get_topic_list(self):
        with AnyReader([self.bagpath], default_typestore=self.typestore) as reader:
            connections = [x for x in reader.connections]
            for connection in connections:
                print(connection.topic)

    def read_imu_data(self):
        time = []
        linear_acc = []
        angular_vel = []
        orientation_quaternion = []
        with AnyReader([self.bagpath], default_typestore=self.typestore) as reader:
            connections = [x for x in reader.connections if x.topic == 'imu/data']
            for connection, timestamp, rawdata in reader.messages(connections=connections):
                msg = reader.deserialize(rawdata, connection.msgtype)
                time.append(msg.header.stamp.sec)
                linear_acc.append([ msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z ])
                angular_vel.append([ msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z ])
                orientation_quaternion.append( [ msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w ]) 
        time = np.asarray(time)
        linear_acc = np.asarray(linear_acc)
        angular_vel = np.asarray(angular_vel)
        orientation_quaternion = np.asarray(orientation_quaternion)

    def read_gnss_data(self):
        time = []
        latitude = []
        latitude_cov = []
        longitude = []
        longitude_cov = []
        altitude = []
        altitude_cov = []   
        with AnyReader([self.bagpath], default_typestore=self.typestore) as reader:
            connections = [x for x in reader.connections if x.topic == 'ublox_position_receiver/fix']
            for connection, timestamp, rawdata in reader.messages(connections=connections):
                msg = reader.deserialize(rawdata, connection.msgtype)
                time.append(msg.header.stamp.sec)
                latitude.append(msg.latitude)
                longitude.append(msg.longitude)
                altitude.append(msg.altitude)
                latitude_cov.append(msg.position_covariance[0])
                longitude_cov.append(msg.position_covariance[4])
                altitude_cov.append(msg.position_covariance[8])
        time = np.asarray(time)
        latitude = np.asarray(latitude)
        latitude_cov = np.asarray(latitude_cov)
        longitude = np.asarray(longitude)
        longitude_cov = np.asarray(longitude_cov) 

    def read_slam_data(self): 
        time = []
        x_coord = []
        y_coord = []
        with AnyReader([self.bagpath], default_typestore=self.typestore) as reader:
            connections = [x for x in reader.connections if x.topic == 'map_base_link/odometry']
            for connection, timestamp, rawdata in reader.messages(connections=connections):
                msg = reader.deserialize(rawdata, connection.msgtype)
                time.append(msg.header.stamp.sec)
                x_coord.append(msg.pose.pose.position.x) 
                y_coord.append(msg.pose.pose.position.y)
        time = np.asarray(time)
        x_coord = np.asarray(x_coord)
        y_coord = np.asarray(y_coord)
        x_coord_0, y_coord_0 = x_coord[0], y_coord[0]
        x_coord, y_coord = x_coord - x_coord_0, y_coord - y_coord_0

    def read_nav_status(self):
        with AnyReader([self.bagpath], default_typestore=self.typestore) as reader:
            connections = [x for x in reader.connections if x.topic == 'move_base/status'] 
            for connection, timestamp, rawdata in reader.messages(connections=connections):
                msg = reader.deserialize(rawdata, connection.msgtype)
                if connection.topic == 'move_base/status':
                    try:
                        status = msg.status_list[0].status
                        if status == 3:
                            # Way point reached
                            pass
                        elif status == 1:
                            # New way point sent to the robot 
                            pass
                    except:
                        pass

    def read_velodyne_data(self):
        time = []
        scans = []
        with AnyReader([self.bagpath], default_typestore=self.typestore) as reader:
            connections = [x for x in reader.connections if x.topic == 'velodyne_points']
            for connection, timestamp, rawdata in reader.messages(connections=connections):
                msg = reader.deserialize(rawdata, connection.msgtype)
                time.append(msg.header.stamp.sec)
                field_names = [f.name for f in msg.fields]
                point_step = msg.point_step
                data = np.frombuffer(msg.data, dtype=np.uint8)
                dtype_list = []
                offset = 0
                for f in msg.fields:
                    if f.datatype == 7:      # FLOAT32
                        dtype_list.append((f.name, np.float32))
                    elif f.datatype == 2:    # UINT8
                        dtype_list.append((f.name, np.uint8))
                    elif f.datatype == 4:    # UINT16
                        dtype_list.append((f.name, np.uint16))
                    offset += f.count * 4  # approximate offset increment
                pc_array = np.frombuffer(msg.data, dtype=np.dtype(dtype_list))
                xyz = np.vstack((pc_array['x'], pc_array['y'], pc_array['z'])).T
                scans.append(xyz)
