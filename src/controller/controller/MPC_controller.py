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
        self.T = 3
        self.N = 40
        self.MODEL = 'ONE_TRACK'  # ONE_TRACK, TWO_TRACK
        self.MPC_OBJECTIVE = 'FOLLOWING'  # EXPLORING, FOLLOWING

        # Load Trackdata
        if self.MPC_OBJECTIVE == 'EXPLORING':
            track_data = load_track("tracks/HRL_centerline.csv")
            #track_data = track_data[::10]    # Apply scaling to x and y columns
            #track_data[:, 0:2] *= 0.1 
            fill1 = np.full((track_data.shape[0], 1), 2.5)
            fill2 = np.full((track_data.shape[0], 1), 2.5)
            track_data = np.hstack((track_data, fill1, fill2))
        elif self.MPC_OBJECTIVE == 'FOLLOWING':
            track_data = load_track("tracks/waypoints.csv")
            #track_data = track_data[::10]  # Apply scaling to x and y columns
            #track_data[:, 0:2] *= 0.1
            fill1 = np.full((track_data.shape[0], 1), 2.5)
            fill2 = np.full((track_data.shape[0], 1), 2.5)
            track_data = np.hstack((track_data, fill1, fill2))

        # Stepsize for Linearization and Optimization
        stepsize_opts = {"stepsize_prep": 0.1,
                        "stepsize_reg": 0.4}

        # Splinify Track
        self.racetrack, self.spline_lengths_raceline = prep_track.prep_track(reftrack_imp=track_data,
                                                                stepsize_opts=stepsize_opts)

        
        # Plot waypoints and generated racetrack
        plot_waypoints_and_track(track_data, self.racetrack)
        
        
        # Load Vehicle and Optimization Parameter
        #pathpath = os.path.join( 'parameter.yaml')
        #with open(pathpath) as stream:
        #    pars = yaml.safe_load(stream)
            
        # Load Vehicle and Optimization Parameter
        #pathpath = os.path.join(os.getcwd(), 'controller', , 'parameter.yaml')
        pathpath = "/sim_ws/install/controller/lib/python3.10/site-packages/controller/parameter.yaml"
        with open(pathpath) as stream:
            pars = yaml.safe_load(stream)
            #/sim_ws/src/controller/controller/parameter.yaml
            #/sim_ws/install/controller/lib/python3.10/site-packages/controller/parameter.yaml
    
        # Get Vehicle Model
        self.model = get_one_track_model(self.racetrack, pars, self.MPC_OBJECTIVE)
        self.x0 = np.array([1, 0, 0, 2.5, 0, 0, 0, 0])
        #self.x0 = np.array([1, 72.65, 9.9, 2.5, 0, 0, 0, 0])
        #self.x0 = np.array([ 1, 0, 0, self.racecar_twist[0], self.racecar_twist[1], self.racecar_angle, 0.8, self.racecar_twist[2] ])

        self.qp_iter = 1

        # Get OCP Structure
        self.ocp = get_OCP(self.model, self.N, self.T, self.x0, self.MODEL)

        self.max_n_sim = 800
        self.end_n = self.max_n_sim
        #self.t_sum = 0
        #self.t_max = 0

        self.nx = self.model.x.size()[0]
        self.nu = self.model.u.size()[0]

        self.i = 1
        #plot
        self.x_hist = np.ndarray((self.nx, self.N, self.max_n_sim))
        self.u_hist = np.ndarray((self.nu, self.N, self.max_n_sim))
        
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

        # 's_min': -0.4189, 's_max': 0.4189, 'sv_min': -3.2, 'sv_max': 3.2,
        
        twist = Twist()
        twist.linear.x = max(min(a, 1.0), -0.5)  # Limit linear velocity
        twist.angular.z = max(min(b, 0.4), -0.4)  # Limit angular velocity
        
        # if twist.angular.z < abs(0.05):
        #     twist.angular.z = 0.0
        self.previous_trans_err = projected_trans_err
        self.previous_rot_err = rot_err
        self.previous_time = current_time
        
        
        if(self.i <= self.max_n_sim):
            '''MPC'''
        
            # Current Position along racetrack - sehr innefizient, aber macht erstmal seinen job
            s_cur, w_cur = amk.path_matching_global(path_cl=self.racetrack[:,0:3], 
                                                    ego_position=np.array([self.racecar_position[1], 
                                                                        self.racecar_position[0] ]) ) #y, x
            mu_ref_idx = np.argmin(np.abs(self.racetrack[:,0] - s_cur))
            mu_ref = self.racetrack[mu_ref_idx, 3]
            mu_cur = 0.1 - mu_ref # heading
            mu_cur = (mu_cur + np.pi) % (2 * np.pi) - np.pi
            
            self.get_logger().info(f"s_cur: {s_cur}, w_cur: {w_cur}, mu_cur: {mu_cur}")

            #x0 = np.array([s_cur, w_cur, mu_cur, v_x, v_y, rotation um z, Gas/Bremssignal [-1;1], Lenkwinkel in rad])
            self.x0 = np.array([ s_cur, w_cur, mu_cur, self.racecar_twist[0], self.racecar_twist[1], self.racecar_angle, 1, self.racecar_twist[2] ])

            # set initial condition
            self.ocp.set(0, "lbx", self.x0)
            self.ocp.set(0, "ubx", self.x0)
        
            # Solve OCP            
            # try:
            #     for j in range(self.qp_iter):
            #         self.ocp.solve()
            # except Exception as e:
            #     self.get_logger().error(f"Error solving OCP: {e}")

            success = self.ocp.solve()
            if success:
                self.get_logger().info(f"OCP solved successfully.")
            else:
                self.get_logger().error(f"Failed to solve OCP.")

            #t_elapsed = time.time() - t


            '''here its going steps '''
            # Save Data in Struct
            # for j in range(self.N):
            #     self.x0 = self.ocp.get(j, "x")
            #     self.u0 = self.ocp.get(j, "u")
            #     self.u0 = self.ocp.get(0, "u")
            #     self.get_logger().info(f"Updated u0: {self.u0}")

                    # Set State for next iteration
            for j in range(self.N):
                self.x0 = self.ocp.get(j, "x")
                self.u0 = self.ocp.get(j, "u")
                for k in range(self.nx):
                    self.x_hist[k, j, self.i] = self.x0[k]
                for k in range(self.nu):
                    self.u_hist[k, j, self.i] = self.u0[k]
            
            self.x0 = self.ocp.get(1, "x")
            self.u0 = self.ocp.get(1, "u")
            
            if self.x0[0] > self.racetrack[-1, 0]:
                self.end_n = self.i
                self.x_hist = self.x_hist[:, :, :self.end_n]
                self.u_hist = self.u_hist[:, :, :self.end_n]
                
        
        self.i += 1
        
        if(self.i >= self.max_n_sim):
            total_track_time = self.end_n * self.T / self.N
            self.get_logger().info("Total track time: {:.3f} s".format(total_track_time))

            # Plot the car path
            plot_track_ros(self.x_hist, self.racetrack)
            
        if(self.i >= self.max_n_sim):
            total_track_time = self.end_n * self.T / self.N
            print("Total track time: {:.3f} s".format(total_track_time))

            # Plot
            if self.MODEL == 'ONE_TRACK':
                keep = plot_track_one_track(self.x_hist, self.racetrack)
                pass
            elif self.MODEL == 'TWO_TRACK':
                keep = plot_track_two_track(self.x_hist, self.u_hist, self.racetrack, self.model)
            


        
        #self.get_logger().info(f'x0: {self.x0}')
        self.get_logger().info(f'x0: {self.x0[7]}, {self.x0[3]}')
        self.get_logger().info(f'u0: {self.u0}')
        self.get_logger().info(f'twist.angular.z: {twist.angular.z}')
        
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

def plot_track_ros(x_hist, racetrack, save_path="car_trajectory3.png"):
    plt.figure(figsize=(10, 6))
    
    # Plot the racetrack
    plt.plot(racetrack[:, 1], racetrack[:, 2], label="Track Centerline", color='black', linewidth=2)
    
    # Plot the car trajectory from x_hist
    x_positions = x_hist[1, 0, :]  # X coordinates 
    y_positions = x_hist[2, 0, :]  # Y coordinates
    plt.plot(x_positions, y_positions, label="Car Path", color='blue', linewidth=2)
    
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