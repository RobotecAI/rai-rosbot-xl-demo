#!/usr/bin/env bash 
. /opt/ros/jazzy/setup.bash

export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=61
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/.fastrtps_conf.xml 
# export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# export CYCLONEDDS_URI=file://$HOME/.cyclonedds_conf.xml

ros2 launch nav2_bringup bringup_launch.py \
    slam:=${SLAM:-True} \
    params_file:=./Examples/navigation/config/navigation_params.yaml \
    map:=./Examples/navigation/maps/map.yaml \
    use_sim_time:=True

