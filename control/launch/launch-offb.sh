#!/bin/bash

killall gzclient
killall gzserver

source ./launch-common.sh

roslaunch control mavros_offb_node.launch
