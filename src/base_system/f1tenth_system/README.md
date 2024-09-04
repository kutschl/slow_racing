# f1tenth_system

This is based on the humble-devel branch of the the official [f1tenth_system](https://github.com/f1tenth/f1tenth_system.git) repository. For further information see the [documentation of F1TENTH](https://f1tenth.readthedocs.io/en/foxy_test/getting_started/firmware/index.html).

## Connect to the car as a client

Please, check if your wifi is connected with `eduroam-cs` or `802.1X`. Unfortunetaly it is not possible to access the car directly. 

Use your cs-account (e.g. `user1`) to build a ssh-connection to one of the pc in the hrl lab room (e.g. `knoppers`).
```
ssh user1@knoppers
```

Now you are able to connect via SSH to the f1tenth car (`pw: f1`). 
```
ssh f1@10.7.4.211
```

Now you should successfully connected with the car. 

## How to drive the car manually

Before you start, please build a ssh connection to the car (see previous step).

### Connect the bluetooth controller
If you want to drive the car manually, you need to connect the car with a bluetooth controller. In our case a dualshock ps4 controller is used. 

To start the pairing mode on the controller, please hold `SHARE` and the PS-Button for some seconds, until the backlight of the controller starts flashing. Now start the bluetooth client on the f1tenth car:

```
bluetoothctl
connect <MAC-ADRESS>
exit
```

Here is the list of the mac adresses of the PS4 dualshock controller:

```
84:17:66:80:91:6A    # black dualshock 'Elfi'
1C:A0:B8:99:1E:68    # gray dualshock 'University'
```

### Run the docker

Follow the commands to run the docker. 

```
cd race_stack/.devcontainer_arm64
chmod +x ./run_container_arm64.sh
./run_container_arm64.sh
```

### Launch
For starting the `briungup_launch`, which is used for driving the car manually with a controller, enter:
```
source install/setup.bash
ros2 launch f1tenth_stack bringup_launch.py
```

### Deadman's switch
On PS4 Dualshock, the L1 button is the deadman's switch for teleop, and the R1 button is the deadman's switch for navigation. You can also remap buttons. See how on the readthedocs documentation.

## Topics

### Topics that the driver stack subscribe to
- `/drive`: Topic for autonomous navigation, uses `AckermannDriveStamped` messages.

### Sensor topics published by the driver stack
- `/scan`: Topic for `LaserScan` messages.
- `/odom`: Topic for `Odometry` messages.
- `/sensors/imu/raw`: Topic for `Imu` messages.
- `/sensors/core`: Topic for telemetry data from the VESC

## External Dependencies

1. ackermann_msgs [https://index.ros.org/r/ackermann_msgs/#foxy](https://index.ros.org/r/ackermann_msgs/#foxy).
2. urg_node [https://index.ros.org/p/urg_node/#foxy](https://index.ros.org/p/urg_node/#foxy). This is the driver for Hokuyo LiDARs.
3. joy [https://index.ros.org/p/joy/#foxy](https://index.ros.org/p/joy/#foxy). This is the driver for joysticks in ROS 2.
4. teleop_tools  [https://index.ros.org/p/teleop_tools/#foxy](https://index.ros.org/p/teleop_tools/#foxy). This is the package for teleop with joysticks in ROS 2.
5. vesc [GitHub - f1tenth/vesc at ros2](https://github.com/f1tenth/vesc/tree/ros2). This is the driver for VESCs in ROS 2.
6. ackermann_mux [GitHub - f1tenth/ackermann_mux: Twist multiplexer](https://github.com/f1tenth/ackermann_mux). This is a package for multiplexing ackermann messages in ROS 2.

## Package in this repo

1. f1tenth_stack: maintains the bringup launch and all parameter files

## Nodes launched in bringup

1. joy
2. joy_teleop
3. ackermann_to_vesc_node
4. vesc_to_odom_node
5. vesc_driver_node
6. urg_node
7. ackermann_mux

## Parameters and topics for dependencies

### vesc_driver

1. Parameters:
   - duty_cycle_min, duty_cycle_max
   - current_min, current_max
   - brake_min, brake_max
   - speed_min, speed_max
   - position_min, position_max
   - servo_min, servo_max
2. Publishes to:
   - sensors/core
   - sensors/servo_position_command
   - sensors/imu
   - sensors/imu/raw
3. Subscribes to:
   - commands/motor/duty_cycle
   - commands/motor/current
   - commands/motor/brake
   - commands/motor/speed
   - commands/motor/position
   - commands/servo/position

### ackermann_to_vesc

1. Parameters:
   - speed_to_erpm_gain
   - speed_to_erpm_offset
   - steering_angle_to_servo_gain
   - steering_angle_to_servo_offset
2. Publishes to:
   - ackermann_cmd
3. Subscribes to:
   - commands/motor/speed
   - commands/servo/position

### vesc_to_odom

1. Parameters:
   - odom_frame
   - base_frame
   - use_servo_cmd_to_calc_angular_velocity
   - speed_to_erpm_gain
   - speed_to_erpm_offset
   - steering_angle_to_servo_gain
   - steering_angle_to_servo_offset
   - wheelbase
   - publish_tf
2. Publishes to:
   - odom
3. Subscribes to:
   - sensors/core
   - sensors/servo_position_command

### throttle_interpolator

1. Parameters:
   - rpm_input_topic
   - rpm_output_topic
   - servo_input_topic
   - servo_output_topic
   - max_acceleration
   - speed_max
   - speed_min
   - throttle_smoother_rate
   - speed_to_erpm_gain
   - max_servo_speed
   - steering_angle_to_servo_gain
   - servo_smoother_rate
   - servo_max
   - servo_min
   - steering_angle_to_servo_offset
2. Publishes to:
   - topic described in rpm_output_topic
   - topic described in servo_output_topic
3. Subscribes to:
   - topic described in rpm_input_topic
   - topic described in servo_input_topic
