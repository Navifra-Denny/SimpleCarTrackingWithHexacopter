#!/bin/bash

killall gzserver
killall gzclient

offboard_control_path="$(rospack find offboard_control)"

file="$offboard_control_path/settings/car_settings.json"
cp $file ~/Documents/AirSim/settings.json

common_file="$offboard_control_path/launch/launch-common.sh"
source $common_file

roslaunch offboard_control gazebo-offb_node.launch