#!/bin/bash

control_path="$(rospack find control)"

file="$control_path/settings/multirotor_px4_settings.json"
cp $file ~/Documents/AirSim/settings.json

px4_dir=/home/ys/git_ws/PX4/PX4-Autopilot/

source $px4_dir/Tools/setup_gazebo.bash $px4_dir $px4_dir/build/px4_sitl_default

export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$px4_dir
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$px4_dir/Tools/sitl_gazebo

roslaunch control airsim-offb_node.launch

