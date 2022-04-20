#!/bin/bash

source /opt/ros/foxy/setup.bash

colcon build;

echo "BUILD COMPLETE"

source install/local_setup.bash;

echo "SOURCE COMPLETE"

cd /home/ubuntu/ros2_ws/src/motor_control/launch

echo "Changed dir"

$(ros2 launch wilie_awaken_launch.py) &

echo "Launched"
