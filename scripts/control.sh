#!/bin/bash

tracking_pkg_path="$(rospack find tracking)"
config_path="${tracking_pkg_path}/../perception_config"

cd ${config_path}
cp config ~/.config/terminator/config

terminator -l perception
