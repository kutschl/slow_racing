# F1TENTH Gym Ros 

## Launching the simulation

Navigate in terminal into `/sim_ws` and enter the following:

```bash
source install/setup.bash

# for launching the basic simulation
ros2 launch f1tenth_gym_ros bringup_launch.py

# for launching the full race stack in simulation (with global planner and controller)
ros2 launch f1tenth_gym_ros race_stack_launch.py
```

**@Samir:** please add your nodes from the controller module in ```race_stack_launch.py``` 

## Parameters for simulation
- The configuration file for the simulation is at `f1tenth_gym_ros/config/sim.yaml`.
- Topic names and namespaces can be configured but is recommended to leave unchanged. 
- The `num_agent` parameter is always set to 1.


## Configure the map
Change the `map` parameter in the configuration file for the simulation `sim.yaml`
```bash 
bridge:
  map: 
    # here you can change the map 
    name: 'Spielberg' 

  ros__parameters:
    ...
```

## Update changes 

After changing the code oder configuration files, please run:
```bash
colcon build
source install/setup.bash
```


## Topics published by the simulation

`/racecar/scan`: The laser scan of the robot

`/racecar/odom`: The odometry of the robot

`/racecar/odom`: The estimated robot pose with AMCL

`/map`: The map of the environment

`/tf` and `/tf_static`: Transformations

## Topics subscribed by the simulation

`/racecar/drive`: The drive command via `AckermannDriveStamped` messages


## Keyboard Teleop

The keyboard teleop node from `teleop_twist_keyboard` is also installed as part of the simulation's dependency. To enable keyboard teleop, set `kb_teleop` to `True` in `sim.yaml`. After launching the simulation, in another terminal, run:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
Then, press `i` to move forward, `u` and `o` to move forward and turn, `,` to move backwards, `m` and `.` to move backwards and turn, and `k` to stop in the terminal window running the teleop node.
