#!/bin/bash
colcon build --symlink-install
source install/setup.bash
# ros2 launch clearpath_gz simulation.launch.py robot_config_yaml:=husky_a200_sample.yaml