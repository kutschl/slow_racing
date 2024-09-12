#!/bin/bash
colcon build 
source install/setup.bash
ros2 launch f1tenth_stack PID_MPC_eff_launch.py