#!/bin/bash
export TURTLEBOT3_MODEL=burger
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash

ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py gui:=false &
sleep 8

ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=true &
sleep 5

ros2 launch nav2_bringup navigation_launch.py \
  slam:=False \
  use_sim_time:=True \
  params_file:=/workspace/src/tbot3_nav_monitor/config/params.yaml &
sleep 8

ros2 launch tbot3_nav_monitor tbot3_nav_monitor.launch.py