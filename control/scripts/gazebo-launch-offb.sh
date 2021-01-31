#!/bin/bash

killall gzserver
killall gzclient

control_path="$(rospack find control)"

file="$control_path/settings/car_settings.json"
cp $file ~/Documents/AirSim/settings.json

common_file="$control_path/launch/launch-common.sh"
source $common_file

roslaunch control gazebo-offb_node.launch