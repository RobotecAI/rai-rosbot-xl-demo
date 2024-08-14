#!/bin/bash

. /opt/ros/${ROS_DISTRO}/setup.bash
ros2 launch nav2_bringup bringup_launch.py \
    slam:=${SLAM:-True} \
    params_file:=./Examples/navigation/config/navigation_params_${ROS_DISTRO}.yaml \
    map:=./Examples/navigation/maps/map.yaml \
    use_sim_time:=True
