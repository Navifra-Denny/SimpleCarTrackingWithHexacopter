#!/bin/bash

if [ "$#" -ne 1 ]; then
    echo "Please input argument \"airsim\" or \"mavros\""
    exit 0
fi

if [ "$1" == "airsim" ]; then
    input="airsim"
elif [ "$1" == "mavros" ]; then
    input="mavros"
else
    echo "Please input argument \"airsim\" or \"mavros\""
    exit 0
fi

if [ $input == "airsim" ]; then
    roslaunch control control.launch do_airsim:="true"
elif [ $input == "mavros" ]; then
    killall gzclient
    killall gzserver

    source ./launch-common.sh

    roslaunch control control.launch do_airsim:="false"
fi