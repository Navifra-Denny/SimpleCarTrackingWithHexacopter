#!/bin/bash

killall gzclient
killall gzserver

source ./launch-common.sh

roslaunch control offb_node.launch
