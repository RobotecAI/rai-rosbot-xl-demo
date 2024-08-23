#!/bin/bash

. /opt/ros/${ROS_DISTRO}/setup.bash
ros2 run rviz2 rviz2 -d /opt/ros/${ROS_DISTRO}/share/nav2_bringup/rviz/nav2_default_view.rviz
