# lidar_based_slam_ros2

##  Installation

#### 1. copy the repo and place it inside src then build with

```bash
cd ~/ros2_ws/src
git clone https://github.com/A-Hanie/lidar_based_slam_ros2.git
```
#### 2. Build the Workspace:

```bash
cd ~/ros2_ws
colcon build --symlink-install
```
## Running SLAM

#### 1. Prepare Lidar Data:

Place your Lidar data inside the `~/data/data` directory to ensure it can be accessed by point_cloud_reader node.

#### 2. Launch the Point Cloud Reader:

```bash
ros2 run point_cloud_reader point_cloud_reader.py
```

#### 3. Run SLAM:

```bash
ros2 launch ndt_slam ndt_slam.launch.py
```


## Map saving

#### Save the Map:

Save the map by using the following ROS2 service call:
```bash
ros2 service call /save_map std_srvs/srv/Trigger "{}"
```

