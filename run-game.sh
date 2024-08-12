#!/usr/bin/env bash
. /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/.fastrtps_conf.xml 
xvfb-run ./Project/build/linux/bin/profile/RobotVacuumSample.GameLauncher -bg_ConnectToAssetProcessor=0 --rhi=null
