import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovarianceStamped
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
import math
import sys
import time
import os
import yaml
import numpy as np
import matplotlib.pyplot as plt

from .racing_MPC.load_track import load_track
from .racing_MPC.get_vehicle_model import get_one_track_model, get_two_track_model
from .racing_MPC.get_OCP import get_OCP
from .racing_MPC.plot_functions import plot_track_one_track, plot_track_two_track
from .racing_MPC import prep_track
from .racing_MPC import amk

import subprocess
import threading

from .mpc_core import mpc_core


class MPCController(Node):
    
    def __init__(self):
        super().__init__('MPC_controller')
        self.velocity_cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.05, self.publish_velocity)
        self.goal_sub = self.create_subscription(PoseStamped, '/planner/goal', self.goal_callback, 10)
        self.sub_odom_ = self.create_subscription(Odometry, '/racecar/odom', self.odom_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/racecar/drive', 10)

        self.goal_position = [0.0, 0.0]
        self.racecar_position = [0.0, 0.0]
        self.racecar_angle = 0.0
        self.racecar_twist = [0.0, 0.0, 0.0]
        #self.racecar_Twist = [msg.Twist.Twist.linear.x, msg.Twist.Twist.linear.y, msg.Twist.Twist.angular.x]
        #self.racecar_state = [self.racecar_position, self.racecar_angle, self.racecar_twist]
        
        """PD Controller parameters"""
        self.kp_lin = 1.0  # Proportional gain for linear velocity
        self.kd_lin = 0.1  # Derivative gain for linear velocity
        self.kp_rot = 1.5  # Proportional gain for angular velocity
        self.kd_rot = 0.2  # Derivative gain for angular velocity
        self.previous_trans_err = 0.0  # Previous translational error
        self.previous_rot_err = 0.0  # Previous rotational error
        self.previous_time = self.get_clock().now()  # Previous time
        self.v = 0.0  # Current linear velocity
        self.w = 0.0  # Current angular velocity
        
        """MPC"""
        self.once = 1
        self.mpc = mpc_core(self.racecar_angle, self.racecar_twist)
        sys.stdout.flush()
        self.get_logger().info(f'mpc init success')


    def publish_velocity(self):
        
        current_time = self.get_clock().now()
        dt = (current_time - self.previous_time).nanoseconds / 1e9  # Convert to seconds

        # Compute errors
        forward = [math.cos(self.racecar_angle), math.sin(self.racecar_angle)]
        q = [self.goal_position[0] - self.racecar_position[0], self.goal_position[1] - self.racecar_position[1]]
        norm_q = math.sqrt(q[0]**2 + q[1]**2)
        
        if norm_q < 0.01:  # Dead band
            self.velocity_cmd_pub.publish(Twist())
            return
        
        projected_trans_err = q[0] * forward[0] + q[1] * forward[1]
        rot_err = math.atan2(q[1], q[0]) - self.racecar_angle
        rot_err = (rot_err + math.pi) % (2 * math.pi) - math.pi  # Normalize to [-pi, pi]

        # PD control
        trans_err_derivative = (projected_trans_err - self.previous_trans_err) / dt if dt > 0 else 0.0
        rot_err_derivative = (rot_err - self.previous_rot_err) / dt if dt > 0 else 0.0

        a = self.kp_lin * projected_trans_err + self.kd_lin * trans_err_derivative
        b = self.kp_rot * rot_err + self.kd_rot * rot_err_derivative

        # 's_min': -0.4189, 's_max': 0.4189, 'sv_min': -3.2, 'sv_max': 3.2,
        
        twist = Twist()
        twist.linear.x = max(min(a, 1.0), -0.5)  # Limit linear velocity
        twist.angular.z = max(min(b, 0.4), -0.4)  # Limit angular velocity
        
        # if twist.angular.z < abs(0.05):
        #     twist.angular.z = 0.0
        self.previous_trans_err = projected_trans_err
        self.previous_rot_err = rot_err
        self.previous_time = current_time
    
        self.x0, self.u0 = self.mpc.mpc_solver(self.racecar_angle, self.racecar_twist, self.racecar_position)
        
        self.get_logger().info(f'x0: {self.x0}')
        #self.get_logger().info(f'x0: {self.x0[7]}, {self.x0[3]}')
        self.get_logger().info(f'u0: {self.u0}')
        #self.get_logger().info(f'twist.angular.z: {twist.angular.z}')
        
        """ 1. Initialize MPC, see init
        Looop over
            2. ocp.set("aktueller stand" = x_vector)
            3. ocp.solve
            4. ocp.get(1, "x") lade die prädiktion
        Loop end
        """
        
        # REAL CAR
        ackermann_drive = AckermannDriveStamped()
        ackermann_drive.header.frame_id = 'racecar/base_link'
        ackermann_drive.header.stamp = self.get_clock().now().to_msg()
        ackermann_drive.drive.steering_angle = twist.angular.z
        ackermann_drive.drive.steering_angle_velocity = 0.0
        ackermann_drive.drive.speed = 2.0
        ackermann_drive.drive.acceleration = 0.0
        ackermann_drive.drive.jerk = 0.0
        self.drive_pub.publish(ackermann_drive)
        #self.racecar_state = [self.racecar_position, self.racecar_angle, self.racecar_twist]      
        
    def goal_callback(self, msg: PoseStamped):
        self.goal_position = [msg.pose.position.x, msg.pose.position.y]
        
    def odom_callback(self, msg: Odometry):
        self.racecar_position = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        orientation_q = msg.pose.pose.orientation
        _, _, self.racecar_angle = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        self.racecar_twist = [msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.angular.z]

        #self.get_logger().info(f"Position: {self.racecar_position}, Angle: {self.racecar_angle}, Twist: {self.racecar_twist}")

def plot_track_ros(x_hist, racetrack, car_positions,  save_path="car_trajectory3.png"):
    plt.figure(figsize=(10, 6))
    
    # Plot the racetrack
    plt.plot(racetrack[:, 1], racetrack[:, 2], label="Track Centerline", color='black', linewidth=2)

    plt.scatter(car_positions[:, 0], car_positions[:, 1], label="Car Path", color='green', linewidth=2, zorder=3)
    
    plt.scatter(car_positions[0, 0], car_positions[0, 1], label="Car Path", color='red', linewidth=2, zorder=5)
    
    # Highlight positions where X and Y are both under 5 (blue)
    under_5_indices = np.where((car_positions[:, 0] < 5) & (car_positions[:, 1] < 5))
    plt.scatter(car_positions[under_5_indices, 0], car_positions[under_5_indices, 1], 
                label=f"Under (5,5) {len(under_5_indices)}", color='blue', linewidth=2, zorder=6)
    
    
    plt.title('Car Path on Track')
    plt.xlabel('X position')
    plt.ylabel('Y position')
    plt.legend()
    plt.grid(True)
    
    # Save the figure to a file
    plt.savefig(save_path)
    plt.close()

def plot_waypoints_and_track(waypoints, racetrack, save_path="track_and_waypoints.png"):
    plt.figure(figsize=(10, 6))
    
    # Plot the original waypoints
    plt.scatter(waypoints[:, 0], waypoints[:, 1], color='red', label='Waypoints', zorder=5)
    
    # Plot the generated racetrack
    plt.plot(racetrack[:, 1], racetrack[:, 2], label='Track', color='blue', linewidth=2, zorder=1)
    
    
    plt.title('Waypoints and Generated Track')
    plt.xlabel('X position')
    plt.ylabel('Y position')
    plt.legend()
    plt.grid(True)
    
    # Save the figure to a file or display it
    plt.savefig(save_path)
    plt.show()


def euler_from_quaternion(quat):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    x, y, z, w = quat
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z

def main(args=None):
    rclpy.init(args=args)
    MPC_controller = MPCController()
    rclpy.spin(MPC_controller)        
    MPC_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()