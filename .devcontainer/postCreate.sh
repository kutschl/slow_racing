#!/bin/bash
mkdir -p src
sudo rosdep update
sudo rosdep install --from-paths /sim_ws/src --ignore-src -y
sudo chown -R $(whoami) /sim_ws
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --cmake-clean-cache