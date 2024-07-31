#!/usr/bin/env bash 
. /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=61
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/.fastrtps_conf.xml 

Editor --project-path ./Project
