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
from .racing_MPC.get_vehicle_model import get_one_track_model
from .racing_MPC.get_OCP import get_OCP
from .racing_MPC.plot_functions import plot_track_one_track
from .racing_MPC import prep_track
from .racing_MPC import amk

class MPCController(Node):
    
    def __init__(self):
        super().__init__('MPC_controller')
        self.velocity_cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10) #param
        self.goal_sub = self.create_subscription(PoseStamped, '/planner/goal', self.goal_callback, 10)#param
        self.sub_odom_ = self.create_subscription(Odometry, '/racecar/odom', self.odom_callback, 10)#param
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/racecar/drive', 10)#param

        self.goal_position = [0.0, 0.0]
        self.racecar_position = [0.0, 0.0]
        self.racecar_angle = 0.0
        self.racecar_twist = [2, 0.0, 0.0]
        #self.racecar_Twist = [msg.Twist.Twist.linear.x, msg.Twist.Twist.linear.y, msg.Twist.Twist.angular.x]
        self.racecar_state = [self.racecar_position, self.racecar_angle, self.racecar_twist]
        
        # PD Controller parameters
        self.kp_lin = 1.0  # Proportional gain for linear velocity
        self.kd_lin = 0.1  # Derivative gain for linear velocity
        self.kp_rot = 1.5  # Proportional gain for angular velocity
        self.kd_rot = 0.2  # Derivative gain for angular velocity
        self.previous_trans_err = 0.0  # Previous translational error
        self.previous_rot_err = 0.0  # Previous rotational error
        self.previous_time = self.get_clock().now()  # Previous time
        self.v = 0.0  # Current linear velocity
        self.w = 0.0  # Current angular velocity
        
        '''
        init MPC start 
        '''

        # Parameter
        self.T = 4
        self.N = 50
        self.MODEL = 'ONE_TRACK'  # ONE_TRACK, TWO_TRACK
        self.MPC_OBJECTIVE = 'EXPLORING'  # EXPLORING, FOLLOWING

        # Load Trackdata
        track_data = load_track("/sim_ws/src/controller/controller/racing_MPC/tracks/waypoints.csv")
        fill1 = np.full((track_data.shape[0], 1), 2.5)
        fill2 = np.full((track_data.shape[0], 1), 2.5)
        track_data = np.hstack((track_data, fill1, fill2))


        # Stepsize for Linearization and Optimization
        stepsize_opts = {"stepsize_prep": 0.1,
                        "stepsize_reg": 0.4}

        # Splinify Track
        self.racetrack, self.spline_lengths_raceline = prep_track.prep_track(reftrack_imp=track_data,   stepsize_opts=stepsize_opts)

        pathpath = "/sim_ws/src/controller/controller/racing_MPC/parameter.yaml"
        with open(pathpath) as stream:
            pars = yaml.safe_load(stream)

        # Get Vehicle Model
        self.model = get_one_track_model(self.racetrack, pars, self.MPC_OBJECTIVE)
        
        # Current Position along racetrack - sehr innefizient, aber macht erstmal seinen job
        s_cur, w_cur = amk.path_matching_global(path_cl=self.racetrack[:,0:3], 
                                                ego_position=np.array([self.racecar_position[0], 
                                                                    self.racecar_position[1] ]) ) #y, x
        mu_ref_idx = np.argmin(np.abs(self.racetrack[:,0] - s_cur))
        mu_ref = self.racetrack[mu_ref_idx, 3]
        mu_cur = self.racecar_angle - mu_ref # heading
        mu_cur = (mu_cur + np.pi) % (2 * np.pi) - np.pi
        
        #self.get_logger().info(f"s_cur: {s_cur}, w_cur: {w_cur}, mu_cur: {mu_cur}")

        #x0 = np.array([s_cur, w_cur, mu_cur, v, Gas/Bremssignal [-1;1], Lenkwinkel in rad])
        self.x0_s = np.array([2, 0, 0, 2, 0, 0])
        self.u0_s = np.array([0, 0])
            
        self.qp_iter = 1

        # Get OCP Structure
        self.ocp = get_OCP(self.model, self.N, self.T, self.x0_s, self.MODEL)

        self.max_n_sim = 10000 #####################################################################################
        self.end_n = self.max_n_sim

        self.nx = self.model.x.size()[0]
        self.nu = self.model.u.size()[0]

        self.i = 1
        #plot
        self.x_hist = np.ndarray((self.nx, self.N, self.max_n_sim))
        self.u_hist = np.ndarray((self.nu, self.N, self.max_n_sim))
        self.car_positions = np.empty((self.max_n_sim, 2))
        
        self.timer = self.create_timer(0.05, self.publish_velocity)
        
        '''
        init end
        '''

       
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

        twist = Twist()
        twist.linear.x = max(min(a, 1.0), -0.5)  # Limit linear velocity
        twist.angular.z = max(min(b, 0.4), -0.4)  # Limit angular velocity
        
        # if twist.angular.z < abs(0.05):
        #     twist.angular.z = 0.0
        self.previous_trans_err = projected_trans_err
        self.previous_rot_err = rot_err
        self.previous_time = current_time
        
        v_x = self.racecar_twist[0]
        
        if 2 > 1:
            if(self.i <= self.max_n_sim):
                '''MPC'''
            
                # Current Position along racetrack - sehr innefizient, aber macht erstmal seinen job
                s_cur, w_cur = amk.path_matching_global(path_cl=self.racetrack[:,0:3], 
                                                        ego_position=np.array([self.racecar_position[0], 
                                                                            self.racecar_position[1] ]) ) #y, x
                w_cur = w_cur
                mu_ref_idx = np.argmin(np.abs(self.racetrack[:,0] - s_cur))
                mu_ref = self.racetrack[mu_ref_idx, 3]
                mu_cur = (self.racecar_angle - mu_ref - np.pi/2) # heading
                mu_cur = (mu_cur + np.pi) % (2 * np.pi) - np.pi
                
                x0 = np.array([s_cur, w_cur, mu_cur, self.racecar_twist[0], 0, self.x0_s[5]])
                # self.get_logger().info(f"x_cur: {x0}")
                # set initial condition
                self.ocp.set(0, "lbx", x0)
                self.ocp.set(0, "ubx", x0)
                #self.ocp.set(0, "x", x0)
                # self.ocp.set(0, "lbu", self.u0_s)
                # self.ocp.set(0, "ubu", self.u0_s)
                # self.ocp.set(0, "u", self.u0_s)

                success = self.ocp.solve()
                # self.get_logger().info(f"OCP Status: {success}")


                # Set State for next iteration
                self.x0_s = self.ocp.get(1, "x")
                self.u0_s = self.ocp.get(1, "u")
                # self.get_logger().info(f"Iteration: {self.i}")
                # self.get_logger().info(f"x0_next_pred: {self.x0_s}")
                for j in range(self.N):
                    self.x0 = self.ocp.get(j, "x")
                    self.u0 = self.ocp.get(j, "u")
                    for k in range(self.nx):
                        self.x_hist[k, j, self.i] = self.x0[k]
                    for k in range(self.nu):
                        self.u_hist[k, j, self.i] = self.u0[k]
            
                    
                # Track car's X and Y position over time
                
                self.car_positions[self.i - 1, 0] = self.racecar_position[0] # X position
                self.car_positions[self.i - 1, 1] = self.racecar_position[1]  # Y position                
            
            self.i += 1
        
        
        if(self.i >= self.max_n_sim):
            total_track_time = self.end_n * self.T / self.N
            # self.get_logger().info("Total track time: {:.3f} s".format(total_track_time))
            # Plot the car path
            plot_track_ros(self.x_hist, self.racetrack, self.car_positions)
            
        if(self.i >= self.max_n_sim):
            total_track_time = self.end_n * self.T / self.N
            print("Total track time: {:.3f} s".format(total_track_time))
            keep = plot_track_one_track(self.x_hist, self.u_hist, self.racetrack)

        
        # REAL CAR
        ackermann_drive = AckermannDriveStamped()
        ackermann_drive.header.frame_id = 'racecar/base_link'
        ackermann_drive.header.stamp = self.get_clock().now().to_msg()
        ackermann_drive.drive.steering_angle = self.x0_s[5].astype(float)
        ackermann_drive.drive.steering_angle_velocity = 0.0
        ackermann_drive.drive.speed = self.x0_s[3].astype(float)
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