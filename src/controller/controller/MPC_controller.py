import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovarianceStamped
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import math
import sys
import time
import os
import yaml
import numpy as np
import matplotlib.pyplot as plt
import csv

from .racing_MPC.load_track import load_track
from .racing_MPC.get_vehicle_model import get_one_track_model
from .racing_MPC.get_OCP import get_OCP
from .racing_MPC.plot_functions import plot_track_one_track
from .racing_MPC import prep_track
from .racing_MPC import amk

class MPCController(Node):
    
    def __init__(self):
        super().__init__('MPC_controller')
        
        self.declare_parameter('odom_topic', '/odometry/filtered')
        # self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('drive_topic', '/drive')
        self.declare_parameter('pose_topic', '/amcl_pose')
        self.declare_parameter('use_sim', False)
        self.declare_parameter('initial_speed', 2.0)
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('initial_pose',[31.4, 56.5, 0.0] )  #  [31.4, 56.5, 0.0]  # [72.65, 9.9, 2.85013586]
        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        pose_topic = self.get_parameter('pose_topic').get_parameter_value().string_value
        drive_topic = self.get_parameter('drive_topic').get_parameter_value().string_value
        imu_topic = '/sensors/imu/raw'
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.use_sim = self.get_parameter('use_sim').get_parameter_value().bool_value
        initial_speed = self.get_parameter('initial_speed').get_parameter_value().double_value
        initial_pose = self.get_parameter('initial_pose').get_parameter_value().double_array_value
        
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)
        # if not self.use_sim:
        #     self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, pose_topic, self.pose_callback, 10)
        #     self.imu_sub = self.create_subscription(Imu, imu_topic, self.imu_callback, 10)

        self.drive_pub = self.create_publisher(AckermannDriveStamped, drive_topic, 10)

        self.racecar_position = [initial_pose[0], initial_pose[1]] #  [0.0 ,0.0] #
        self.racecar_angle = 0.0 # initial_pose[2] 
        self.racecar_twist = [initial_speed, 0.0, 0.0] 
        self.steering_angle = 0.0
        
        '''
        init MPC start 
        '''
        # Parameter
        self.T = 4
        self.N = 40
        self.MODEL = 'ONE_TRACK'  # ONE_TRACK, TWO_TRACK
        self.MPC_OBJECTIVE = 'EXPLORING'  # EXPLORING, FOLLOWING

        # Load Trackdata
        track_data = load_track("/sim_ws/src/controller/controller/racing_MPC/tracks/HRL_centerline.csv") #TODO
        # track_data = track_data[::5]
        track_data = track_data / 20.0
        fill1 = np.full((track_data.shape[0], 1), 2.5)
        fill2 = np.full((track_data.shape[0], 1), 2.5)
        track_data = np.hstack((track_data, fill1, fill2))


        # Stepsize for Linearization and Optimization
        stepsize_opts = {"stepsize_prep": 0.4,
                        "stepsize_reg": 0.1}

        # Splinify Track
        self.racetrack, self.spline_lengths_raceline = prep_track.prep_track(reftrack_imp=track_data,   stepsize_opts=stepsize_opts)
        
        # plot_waypoints_and_track(track_data, self.racetrack)
        
        pathpath = "/sim_ws/src/controller/controller/racing_MPC/parameter.yaml" #TODO
        with open(pathpath) as stream:
            pars = yaml.safe_load(stream)

        # Get Vehicle Model
        self.model = get_one_track_model(self.racetrack, pars, self.MPC_OBJECTIVE)
        
        # Current Position along racetrack - sehr innefizient, aber macht erstmal seinen job
        s_cur, w_cur = amk.path_matching_global(path_cl=self.racetrack[:,0:3], 
                                                ego_position=np.array([self.racecar_position[0], 
                                                                    self.racecar_position[1] ]) ) #x, y
        mu_ref_idx = np.argmin(np.abs(self.racetrack[:,0] - s_cur))
        mu_ref = self.racetrack[mu_ref_idx, 3]
        mu_cur = self.racecar_angle - mu_ref # heading
        mu_cur = (mu_cur + np.pi) % (2 * np.pi) - np.pi
        
        #self.get_logger().info(f"s_cur: {s_cur}, w_cur: {w_cur}, mu_cur: {mu_cur}")

        #x0 = np.array([s_cur, w_cur, mu_cur, v, Gas/Bremssignal [-1;1], Lenkwinkel in rad])
        #self.x0_s = np.array([2, 0, 0, 2, 0, 0])
        self.x0_s = np.array([s_cur, w_cur, mu_cur, self.racecar_twist[0], 0, 0])
        
        self.u0_s = np.array([0, 0])
            
        self.qp_iter = 2

        # Get OCP Structure
        self.ocp = get_OCP(self.model, self.N, self.T, self.x0_s, self.MODEL)

        self.max_n_sim = 1500 #####################################################################################
        self.end_n = self.max_n_sim

        self.nx = self.model.x.size()[0]
        self.nu = self.model.u.size()[0]
        
        self.t_sum = 0
        self.t_max = 0

        self.i = 1
        #plot
        self.x_hist = np.ndarray((self.nx, self.N, self.max_n_sim))
        self.u_hist = np.ndarray((self.nu, self.N, self.max_n_sim))
        self.car_positions = np.empty((self.max_n_sim, 2))
        
        self.ackermann_drive = AckermannDriveStamped()
        self.ackermann_drive.header.frame_id = self.base_frame
        self.ackermann_drive.header.stamp = self.get_clock().now().to_msg()
        self.ackermann_drive.drive.steering_angle = 0.0
        self.ackermann_drive.drive.steering_angle_velocity = 0.0
        self.ackermann_drive.drive.speed = 2.0
        self.ackermann_drive.drive.acceleration = 0.5
        self.ackermann_drive.drive.jerk = 0.0
        self.drive_pub.publish(self.ackermann_drive)
        
        sys.stdout.flush()
        self.get_logger().info(f'mpc init success')
        '''
        init MPC end
        '''
        if self.use_sim:
            # Open a CSV file for writing
            self.csv_file = open('mpc_data_15_30_param_.csv', mode='w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            # Write the CSV headers
            self.csv_writer.writerow(['Iteration', 's_cur', 'w_cur', 'mu_cur', 'v', 'angle', 'Racecar X', 'Racecar Y, Racecar heading'])
        
        self.timer = self.create_timer(0.05, self.publish_velocity)

       
    def publish_velocity(self):
            
        if(self.i <= self.max_n_sim):
            '''MPC'''
            t = time.time()
            
            # Current Position along racetrack - sehr innefizient, aber macht erstmal seinen job
            s_cur, w_cur = amk.path_matching_global(path_cl=self.racetrack[:,0:3], 
                                                    ego_position=np.array([self.racecar_position[0], 
                                                                        self.racecar_position[1] ]) ) #y, x
            w_cur = w_cur
            mu_ref_idx = np.argmin(np.abs(self.racetrack[:,0] - s_cur))
            mu_ref = self.racetrack[mu_ref_idx, 3]
            mu_cur = (self.racecar_angle - mu_ref - np.pi/2) # heading
            mu_cur = (mu_cur + np.pi) % (2 * np.pi) - np.pi
            
            #self.get_logger().info(f"drive.steering_angle: {self.ackermann_drive.drive.steering_angle}, {self.x0_s[5]}, {self.steering_angle}")
            
            x0 = np.array([s_cur, w_cur, mu_cur, self.racecar_twist[0], 0, self.steering_angle])
            # self.get_logger().info(f"x_cur: {x0}")
            # set initial condition
            self.ocp.set(0, "lbx", x0)
            self.ocp.set(0, "ubx", x0)

            for j in range(self.qp_iter):
                
                success = self.ocp.solve()
                # self.get_logger().info(f"OCP Status: {success}")
            # if success:
            #     self.get_logger().info(f"OCP solved successfully.")
            # else:
            #     self.get_logger().info(f"Failed to solve OCP.")
            #     #self.ocp.reset()
            #     self.ocp.set(0, "lbx", x0)
            #     self.ocp.set(0, "ubx", x0)
            #     self.ocp.solve()
                    
            t_elapsed = time.time() - t

            

            # Calculate Time Sum
            self.t_sum += t_elapsed
            if t_elapsed > self.t_max:
                self.t_max = t_elapsed

            # Set State for next iteration
            self.x0_s = self.ocp.get(1, "x")
            self.u0_s = self.ocp.get(1, "u")
            # self.get_logger().info(f"Iteration: {self.i}")
            # self.get_logger().info(f"x0_next_pred: {self.x0_s}")
            # for j in range(self.N):
            #     self.x0 = self.ocp.get(j, "x")
            #     self.u0 = self.ocp.get(j, "u")
            #     for k in range(self.nx):
            #         self.x_hist[k, j, self.i] = self.x0[k]
            #     for k in range(self.nu):
            #         self.u_hist[k, j, self.i] = self.u0[k]
        
                
            # Track car's X and Y position over time
            
            self.car_positions[self.i - 1, 0] = self.racecar_position[0] # X position
            self.car_positions[self.i - 1, 1] = self.racecar_position[1]  # Y position                
        


        if self.use_sim:
            self.i += 1
            # Save data to CSV
            self.csv_writer.writerow([self.i, s_cur, w_cur, mu_cur, self.x0_s[3], self.x0_s[5], self.racecar_position[0], self.racecar_position[1], self.racecar_angle])
        
        if(self.i >= self.max_n_sim):
            
            self.csv_file.close()  # Close the CSV file when the run is complete
            
            end_n = self.i
            total_track_time = end_n * self.T / self.N
            print("Total track time: {:.3f} s".format(total_track_time))
            print("Total computation time: {:.3f} s".format(self.t_sum))
            print("Average computation time: {:.3f} ms".format(self.t_sum / end_n * 1000))
            print("Maximum computation time: {:.3f} ms".format(self.t_max * 1000))

            # total_track_time = self.end_n * self.T / self.N
            # plot_track_ros(self.x_hist, self.racetrack, self.car_positions)
            self.i = 1 
            # print("Total track time: {:.3f} s".format(total_track_time))
            # keep = plot_track_one_track(self.x_hist, self.u_hist, self.racetrack)

        
        # REAL CAR
        #ackermann_drive = AckermannDriveStamped()
        self.ackermann_drive.header.frame_id = self.base_frame
        self.ackermann_drive.header.stamp = self.get_clock().now().to_msg()
        self.ackermann_drive.drive.steering_angle = self.x0_s[5].astype(float)
        self.ackermann_drive.drive.steering_angle_velocity = 0.0
        self.ackermann_drive.drive.speed = self.x0_s[3].astype(float)
        self.ackermann_drive.drive.acceleration = 0.0
        self.ackermann_drive.drive.jerk = 0.0
        self.drive_pub.publish(self.ackermann_drive)
            
    def estimate_steering_angle(self, angular_vel_z, linear_vel_x, wheelbase):
        # Check to avoid division by zero
        if linear_vel_x != 0:
            steering_angle = math.atan(wheelbase * angular_vel_z/linear_vel_x)
            return steering_angle
        return 0.0
        
    def odom_callback(self, msg: Odometry):
        self.racecar_position = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        orientation_q = msg.pose.pose.orientation
        _, _, self.racecar_angle = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        self.racecar_twist = [msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.angular.z]
        self.wheelbase = 0.35 
        self.steering_angle = self.estimate_steering_angle(self.racecar_twist[2], self.racecar_twist[0], self.wheelbase)
        #self.get_logger().info(f"Position: {self.racecar_position}, Angle: {self.racecar_angle}, Twist: {self.racecar_twist}")
    
    # def pose_callback(self, msg: PoseWithCovarianceStamped):
    #     self.racecar_position = [msg.pose.pose.position.x, msg.pose.pose.position.y]
    #     orientation_q = msg.pose.pose.orientation
    #     _, _, self.racecar_angle = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

    # def imu_callback(self, msg: Imu):
    #     # imu : blabblabal
    #     self.racecar_imu = [msg.orientation.z, msg.angular_velocity.z, msg.linear_acceleration.x]
    #     self.wheelbase = 0.35 
    #     self.steering_angle = self.estimate_steering_angle(msg.angular_velocity.z, self.racecar_twist[0], self.wheelbase)
        
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
    
    # Plot the generated racetrack
    plt.plot(racetrack[:, 1], racetrack[:, 2], label='Track', color='blue', linewidth=2, zorder=6)
    
    # Plot the original waypoints
    plt.scatter(waypoints[:, 0], waypoints[:, 1], color='red', label='Waypoints', zorder=5)
    
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