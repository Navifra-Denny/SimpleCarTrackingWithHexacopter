#!/bin/bash

control_pkg_path="$(rospack find control)"
config_path="${control_pkg_path}/../config"

cd ${config_path}
cp config ~/.config/terminator/config

terminator -l waypoints_following
