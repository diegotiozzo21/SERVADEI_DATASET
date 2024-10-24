# SERVADEI_DATASET

This open-source dataset that can be used to test and develop novel SLAM algorithms and mapping techniques for autonomous mobile robots in vineyards. The dataset consists of raw ROS bag files and multispectral images collected in the field with the robot autonomously inspecting the vineyard rows. The dataset includes information on the robot status, IMU, GNSS, and wheels odometry data, point clouds of the vineyard, and multispectral images.

![](friulbot.jpg)

In the following table we describe all the topics stored in the ROS bag files. 

| Topic                                | Message Type | Rate [Hz] | Description                                                                                                                                                  |
|--------------------------------------|--------------|-----------|--------------------------------------------------------------------------------------------------------------------------------------------------------------|
| velodyne\_points                     | PointCloud2  | 10        | LiDAR scans of the Velodyne sensor.                                                                                                                           |
| imu/data                             | Imu          | 200       | Linear accelerations, angular velocities, and orientation of the IMU.                                                                                         |
| ublox\_position\_receiver/fix         | NavSatFix    | 3         | GNSS measurements.                                                                                                                                            |
| odom                                 | Odometry     | 50        | Odometry from the encoders of the wheels.                                                                                                                     |
| scout\_status                        | ScoutStatus  | 50        | Driver voltage and temperature, motor currents, velocities [rpm], and temperatures, battery voltage, current, temperature, state of charge (SOC), state of health (SOH). |
| micasense\_capture\_lidar\_points     | PointCloud2  | 0.5       | 3D LiDAR scans captured simultaneously with the image acquisition by the Micasense camera.                                                                    |
| map\_base\_link/odometry             | Odometry     | 20        | Pose of the robot tracked by the SLAM algorithm.                                                                                                              |
| tf                                   | tfMessage    | 400       | Transformations (translations and rotations) between different coordinate frames in the robot.                                                                |


The dataset comprises a file indicating the pose of each sensor with respect to the LiDAR sensor. The multispectral images are stored as TIFF files in separate directories for each survey. The data from the DLS2 light sensor are available as the metadata associated to each TIFF image of the multispectral camera. Additionally, the dataset includes the post-processed ROS bag files in which data from the Velodyne LiDAR sensor and the Micasense camera are merged together to compute the vegetation indexes. This dataset provides a resource to facilitate advancements in autonomous navigation, SLAM, and precision agriculture, by offering both raw and post-processed data.

## Test 1
[Download](https://zenodo.org/records/13983627?token=eyJhbGciOiJIUzUxMiJ9.eyJpZCI6ImExZjQ1NWRmLTY4M2UtNDcyMy1iNTAzLTliOWFhODMwYjUxZCIsImRhdGEiOnt9LCJyYW5kb20iOiI2NDQ0YmExZTExMGVkODA4ZDllY2NjNzJkMDJmN2E4MiJ9.WBCjiIw4l4HL7RD5h0HBXe4a8c7NZijiVUWVPciTRpCN4qtiwPdKDcBX_uMncdLZsy-Kbfq9mOQjdIC0B3khkA)

## Test 2 

### 22-07-2024
[Download](https://zenodo.org/records/13983719?token=eyJhbGciOiJIUzUxMiJ9.eyJpZCI6IjVkNDNmOTQxLTRhOTAtNDZiNi05OWQ4LWQwMmU1NWFiZGUzNyIsImRhdGEiOnt9LCJyYW5kb20iOiIzYzk2MzJjYTIxNjAwN2M2NzNiOGFmMzgzYTg4NjYyOSJ9.GroOJDeuzUqFtu9qklvMAJU9bNgnC82H58sU1e9VoxoMEhiJYJyzF2Cw8-V5R2QnQuDFpHD_azHQwF7BqRa7CQ)

### 23-07-2024
[Download](https://zenodo.org/records/13984245?token=eyJhbGciOiJIUzUxMiJ9.eyJpZCI6IjhjZWFlMjQwLTg0ZjMtNGMwZS05MTA1LTUwMGQxZDQxYWI4NyIsImRhdGEiOnt9LCJyYW5kb20iOiJjYWM2NTY0NjUyZWFhYTgwNGMwYWFmYjY4YzhjMzFkYiJ9.8VBvKSNPeOybSzPxjptQ8q-WYVyyvpWsZwOZKzh58yoDBN5j4h8rMYXjBghfi6VQSf4LDs4FJ4iscaGEMy-kCQ)

### 25-07-2024
[Download](https://zenodo.org/records/13984309?token=eyJhbGciOiJIUzUxMiJ9.eyJpZCI6IjVjZWY4NGM3LWE4ZGMtNGM0NS04NTdlLTY3MWYwM2FkNjFlMiIsImRhdGEiOnt9LCJyYW5kb20iOiIzN2VkYmIzOWRjNTNlZmNkNzRmZGIwYTI4NWY2OTExYiJ9.fWafijSYQXUzAjo39pwBRtodcIIWTEjv39eGGUAzcDZUMW7XSzMHmpjx0bSYU28KwujhlxFNjp1s8PSR6yaFYA)

## Test 3
[Download](https://zenodo.org/records/13984355?token=eyJhbGciOiJIUzUxMiJ9.eyJpZCI6IjkzZWQ3MDFjLTI0OGEtNDU1Zi1iZDQ2LTI0MzBiYjA4ZTc2MCIsImRhdGEiOnt9LCJyYW5kb20iOiJjYmYwYjEwZTY1MzZkNGUwYWIyZDZhZWMwOWVlZjBkMCJ9.OfTHa9puoNne9wjBi7tWgw8oHgD1hLrbxib9QkmYI1HIniHyq-HEU4-Xh9yv2jzd_OQSzxxiSl_H4Xp4ozjOQA)


