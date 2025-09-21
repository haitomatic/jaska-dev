# Jaska robot development environment
## Build
```
# on target
docker build -t ghcr.io/haitomatic/jaska-dev:base .
docker build -t ghcr.io/haitomatic/jaska-dev:lidar .
```

## Run
```
# ZED BOX
docker run -it --rm --name jaska_lidar --network host --privileged -v ~/haito_dev:/home/jetson/haito_dev -e ROS_DOMAIN_ID=1 -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp -e ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET -v /dev:/dev ghcr.io/haitomatic/jaska-dev:base bash

# PC
export ROS_DOMAIN_ID=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
rviz2
```
