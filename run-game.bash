#!/bin/bash

. /opt/ros/${ROS_DISTRO}/setup.bash
./Project/build/linux/bin/profile/RAIROSBotXLDemo.GameLauncher -bg_ConnectToAssetProcessor=0 --rhi=null
