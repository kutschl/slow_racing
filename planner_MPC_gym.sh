#!/bin/bash
colcon build 
source install/setup.bash
ros2 launch f1tenth_gym_ros planner_MPC_launch.py