#!/bin/bash

if [ "$#" -ne 1 ]; then
    echo "Try 'launch-visual.sh -h' for more information."
    exit 0
else
    if [ "$#" -eq 1 ]; then
        if ! [ "$1" == "-h" ] && ! [ "$1" == "-r" ] && ! [ "$1" == "-s" ]; then
            echo "Try 'launch-visual.sh -h' for more information."
            exit 0
        elif [ "$1" == "-r" ]; then
            echo "1 = -r"
        elif [ "$1" == "-s" ]; then
            echo "1 = -s"
        fi
    fi
fi

simulation="false"

if [ "$1" == "-h" ]; then
    echo -e "Usage: launch-visual.sh [OPTION]\n"
    echo "  -h, show this help message and exit."
    echo "  -s, RViz with parameter use_sim_time is true"
    echo "  -r, RViz with parameter use_sim_time is false"
elif [ "$1" == "-r" ]; then
    simulation="false"
elif [ "$1" == "-s" ]; then
    simulation="true"
fi

roslaunch visualization visualize_rviz.launch simulation:="$simulation"