# Jaska robot development environment
## Build
Dockerfile is based on https://github.com/atinfinity/l4t-ros2-docker/tree/main/jazzy
```
# on target
docker build -t ghcr.io/haitomatic/jaska-dev:base .
docker build -t ghcr.io/haitomatic/jaska-dev:lidar .
```

## Usage
### Lidar
```
# ZED BOX
docker run -it --rm --name jaska_lidar --network host --privileged -v ~/haito_dev:/home/jetson/haito_dev -e ROS_DOMAIN_ID=1 -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp -e ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET -v /dev:/dev ghcr.io/haitomatic/jaska-dev:base bash

# PC
export ROS_DOMAIN_ID=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
rviz2 /home/haito/haito_dev/ros2_ws/jaska-dev/lidar_utils/unilidar_lidar.rviz

# record and playback
docker run -d --name jaska_lidar --network host --privileged -v ~/haito_dev:/home/jetson/haito_dev -e ROS_DOMAIN_ID=1 -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST -v /dev:/dev ghcr.io/haitomatic/jaska-dev:base bash -c "source /home/jetson/haito_dev/ros2_ws/install/setup.bash && cd /home/jetson/haito_dev/jaska-dev/lidar_utils && ros2 launch unitree_lidar_ros2 launch.py record_bag:=true"
docker kill --signal=SIGINT jaska_lidar
ros2 bag play unitree_l2_mapping_data
ros2 bag info unitree_l2_mapping_data

# Basic LIO
ros2 bag play unitree_l2_mapping_data
ros2 launch point_lio mapping_unilidar_l2.launch.py
```

### Mapping
#### Installation
*Install GTSAM (4.1.1)*
```bash
wget -O gtsam.zip https://github.com/borglab/gtsam/archive/refs/tags/4.1.1.zip
unzip gtsam.zip
cd gtsam-4.1.1/
mkdir build && cd build
cmake -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DGTSAM_USE_SYSTEM_EIGEN=ON ..
sudo make install -j16
```
*Install TEASER++*
```bash
git clone https://github.com/MIT-SPARK/TEASER-plusplus.git
cd TEASER-plusplus && mkdir build && cd build
cmake .. -DENABLE_DIAGNOSTIC_PRINT=OFF
sudo make install -j16
sudo ldconfig
```
*Install TBB*
```bash
sudo apt install -y libtbb-dev
```
*Upstream components*
```
git clone https://github.com/haitomatic/Localization-QN.git
git clone https://github.com/haitomatic/SAM-QN.git
git clone https://github.com/illusionaryshelter/Quatro.git
git clone https://github.com/illusionaryshelter/nano_gicp.git
```

#### Build
```
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

#### Execution
```
source install/setup.bash
ros2 launch jaska_dev mapping.launch.py
# another terminal
ros2 bag play <path_to_ros2_bag>
```


### Joystick controller
