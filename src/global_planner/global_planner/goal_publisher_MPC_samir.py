import rclpy
from rclpy.node import Node
import os 
from ament_index_python.packages import get_package_share_directory
import yaml 
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
import pandas as pd
import numpy as np
import math
from ackermann_msgs.msg import AckermannDriveStamped
from tf_transformations import quaternion_from_euler, euler_from_quaternion


class GoalPublisherMPCSamir(Node):
    def __init__(self):
        super().__init__('goal_publisher_MPC_samir')
        
        # Parameter
        self.declare_parameter('map_name', 'HRL')
        self.declare_parameter('goal_topic', '/goal')
        self.declare_parameter('namespace', '/planner')
        self.declare_parameter('waypoints_mode', 'centerline')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('pose_topic', '/amcl_pose')
        self.declare_parameter('drive_topic', '/drive')
        self.declare_parameter('publish_drive', True)
        self.declare_parameter('min_goal_distance', 0.50) # 1.00
        self.declare_parameter('waypoints_step_size', 5) # 20
        self.declare_parameter('use_slam_pose', True)
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('steering_pid_kp', 0.30) # 0.5
        self.declare_parameter('steering_pid_ki', 0.00) # 0.0
        self.declare_parameter('steering_pid_kd', 0.10) # 0.1
        self.declare_parameter('drive_speed', 2.0)
        
        map_name = self.get_parameter('map_name').get_parameter_value().string_value
        namespace = self.get_parameter('namespace').get_parameter_value().string_value
        goal_topic = f'{namespace}{self.get_parameter("goal_topic").get_parameter_value().string_value}'
        waypoints_mode = self.get_parameter('waypoints_mode').get_parameter_value().string_value
        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        pose_topic = self.get_parameter('pose_topic').get_parameter_value().string_value
        drive_topic = self.get_parameter('drive_topic').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.map_frame = self.get_parameter('map_frame').get_parameter_value().string_value
        waypoints_step_size = self.get_parameter('waypoints_step_size').get_parameter_value().integer_value 
        self.min_goal_distance = self.get_parameter('min_goal_distance').get_parameter_value().double_value
        use_slam_pose = self.get_parameter('use_slam_pose').get_parameter_value().bool_value
        publish_drive = self.get_parameter('publish_drive').get_parameter_value().bool_value
        self.steering_pid_kp = self.get_parameter('steering_pid_kp').get_parameter_value().double_value
        self.steering_pid_ki = self.get_parameter('steering_pid_ki').get_parameter_value().double_value
        self.steering_pid_kd = self.get_parameter('steering_pid_kd').get_parameter_value().double_value
        self.drive_speed = self.get_parameter('drive_speed').get_parameter_value().double_value
        self.drive_steering_angle = 0.0
        self.racecar_twist = [1.5, 0.0, 0.0]
        # Load map config
        map_config_path = os.path.join(get_package_share_directory('global_planner'), 'maps', map_name, f'{map_name}_map.yaml')
        with open(map_config_path, 'r') as file:
            map_config_dict = yaml.safe_load(file)
                
        # Waypoints: centerline (option 1)
        if waypoints_mode == 'centerline':
            # load centerline
            centerline_path = os.path.join(get_package_share_directory('global_planner'), 'maps', map_name, f'{map_name}_centerline_mpc.csv')
            centerline_csv = pd.read_csv(centerline_path)
            centerline_xy = centerline_csv[['x', 'y', 'v', 'r']].to_numpy()
            centerline_xy = centerline_xy[::waypoints_step_size]
            # centerline_xy = centerline_xy*float(map_config_dict['resolution'])
            # compute theta
            centerline_theta = np.zeros(shape=(centerline_xy.shape[0], 1))
            for i in range(centerline_theta.shape[0]):
                if i == centerline_theta.shape[0]-1:
                    x2 = centerline_xy[0][0]
                    y2 = centerline_xy[0][1]
                else:
                    x2 = centerline_xy[i+1][0]
                    y2 = centerline_xy[i+1][1]
                x1 = centerline_xy[i][0]
                y1 = centerline_xy[i][1]
                centerline_theta[i] = math.atan2(y2-y1, x2-x1)
            # combine xy and theta for centerline
            centerline = np.hstack((centerline_xy, centerline_theta))
            self.goals = centerline.copy()
            """
            # # Smooth the path (calls the smooth_path function)
            # self.smooth_path()
            
            # self.plot_path_comparison(centerline, self.goals)

            # Optional: Print original vs smoothed waypoints for comparison
            # self.print_path_comparison(centerline, self.goals)
            """
        
        # TODO: Waypoints: raceline (option 2)
        else: 
            pass 
        
        # Init goal
        self.goal_idx = int(map_config_dict['starting_index']/waypoints_step_size)+1
        self.goal_distance = np.inf
        
        # Init car pose
        self.car_pose = np.array(map_config_dict['starting_pose'])
        
        # Init PID controller for steering
        self.error_sum = 0.0
        self.last_error = 0.0
        self.previous_trans_err = 0.0
        # Subscriber and publisher
        if use_slam_pose: 
            self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, pose_topic, self.pose_callback, 10)
        else:
            self.odom_sub = self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)
               
        self.goal_pub = self.create_publisher(PoseStamped, goal_topic, 10)
        self.goal_timer = self.create_timer(0.01, self.goal_callback)
        
        if publish_drive:
            self.drive_pub = self.create_publisher(AckermannDriveStamped, drive_topic, 10)
            self.drive_timer = self.create_timer(0.05, self.drive_callback)   
        
    def pose_callback(self, msg: PoseWithCovarianceStamped):
        # update car position from pose
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        _,_,theta = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        self.car_pose = np.array([x,y,theta])
    
    """
    def plot_path_comparison(self, original_goals, smoothed_goals):
        plt.figure(figsize=(10, 6))
        plt.plot(original_goals[:, 0], original_goals[:, 1], 'ro-', label="Original Path")
        plt.plot(smoothed_goals[:, 0], smoothed_goals[:, 1], 'bo-', label="Smoothed Path")
        plt.legend()
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.title("Original vs Smoothed Path")
        plt.grid(True)
        plt.show()"""
    """
    def smooth_path(self):
        from scipy.interpolate import CubicSpline
        
        # Extract x and y coordinates from the goals
        x_coords = self.goals[:, 0]
        y_coords = self.goals[:, 1]

        # Create a cubic spline interpolator
        cs = CubicSpline(x_coords, y_coords)

        # Generate new smoothed points (you can choose how dense the points should be)
        smooth_x = np.linspace(x_coords[0], x_coords[-1], num=len(x_coords) * 10)
        smooth_y = cs(smooth_x)

        # Combine the smoothed x, y and add the theta values back to the goals
        smooth_theta = np.zeros_like(smooth_x)
        for i in range(len(smooth_x) - 1):
            smooth_theta[i] = math.atan2(smooth_y[i+1] - smooth_y[i], smooth_x[i+1] - smooth_x[i])

        # Final smoothed goals: Combine x, y, and theta
        smooth_goals = np.column_stack((smooth_x, smooth_y, np.zeros_like(smooth_x), smooth_theta))
        
        # Replace the current goals with the smoothed ones
        self.goals = smooth_goals
    """
    def odom_callback(self, msg: Odometry):
        # update car position from odom
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        _,_,theta = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        self.car_pose = np.array([x,y,theta])
        self.racecar_twist = [msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.angular.z]
        
    def goal_callback(self):
        # update distance between car and goal
        self.goal_distance = np.linalg.norm((self.car_pose[0:2]-self.goals[self.goal_idx][0:2]))
        
        # Find the direction to the next goal
        direction_to_goal = math.atan2(self.goals[self.goal_idx][1] - self.car_pose[1], 
                                        self.goals[self.goal_idx][0] - self.car_pose[0])

        # Calculate the difference between the car's current heading and the direction to the goal
        heading_diff = direction_to_goal - self.car_pose[2]
        heading_diff = (heading_diff + math.pi) % (2 * math.pi) - math.pi  # Normalize the angle

        # check goal
        if self.goal_distance < self.min_goal_distance or abs(heading_diff) > (math.pi / 2):
            self.goal_idx +=1
            if self.goal_idx >= len(self.goals):
                self.goal_idx = 0
        # publish goal
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = self.map_frame
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.position.x = self.goals[self.goal_idx][0]
        goal_msg.pose.position.y = self.goals[self.goal_idx][1]
        _, _, z, w =  quaternion_from_euler(ai=0.0, aj=0.0, ak=self.goals[self.goal_idx][4]) # theta
        goal_msg.pose.orientation.w = w
        goal_msg.pose.orientation.z = z
        self.goal_pub.publish(goal_msg)
        
    # Calculate average curvature between the current and the next 10 points
    def calculate_curvature(self, N = 10):
        lookahead_points = N    # Number of points to use for curvature calculation
        total_curvature = 0.0
        valid_points = 0

        for i in range(self.goal_idx, min(self.goal_idx + lookahead_points, len(self.goals) - 1)):
            dx = self.goals[i + 1][0] - self.goals[i][0]
            dy = self.goals[i + 1][1] - self.goals[i][1]
            current_theta = math.atan2(dy, dx)

            if i > self.goal_idx:
                # Calculate the difference between consecutive angles
                previous_theta = math.atan2(self.goals[i][1] - self.goals[i - 1][1], self.goals[i][0] - self.goals[i - 1][0])
                angle_diff = abs(current_theta - previous_theta)
                angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi  # Normalize to [-pi, pi]

                total_curvature += abs(angle_diff)
                valid_points += 1

        # Return average curvature across valid points (if any valid points)
        if valid_points > 0:
            return total_curvature / valid_points
        else:
            return 0.0  # No curvature if there aren't enough valid points
  
    def drive_callback(self):
        # steering PID controller 
        dx = self.goals[self.goal_idx][0]-self.car_pose[0]
        dy = self.goals[self.goal_idx][1]-self.car_pose[1]
        theta_error = math.atan2(dy,dx) - self.car_pose[2]
        theta_error = (theta_error + math.pi) % (2 * math.pi)  - math.pi
        
        # Add a deadband for small errors (e.g., below 0.02 radians)
        if abs(theta_error) < 0.00:
            theta_error = 0.0
        # self.last_error = theta_error
        # self.error_sum += theta_error
        # delta_error = theta_error - self.last_error
        
        # Calculate average curvature over the next 10 points
        curvature = self.calculate_curvature(10)
        
        current_speed = self.racecar_twist[0]
        
        # Adjust PID gains dynamically based on curvature and speed
        # Reduce steering corrections on straight paths, increase on curves
        if curvature > 0.2:  
            dynamic_kp = self.steering_pid_kp #* (1 + curvature * 5)
            dynamic_kd = self.steering_pid_kd #* (1 + curvature * 2)
        else:
            self.get_logger().info(f'curve: {curvature}, Pose: {self.car_pose}')
            dynamic_kp = self.steering_pid_kp * 0.5
            dynamic_kd = self.steering_pid_kd * 0.5
        
        # PID controller for steering
        delta_error = theta_error - self.last_error
        self.error_sum += theta_error
        pid_correction = (dynamic_kp * theta_error) + (self.steering_pid_ki * self.error_sum) + (dynamic_kd * delta_error)

        # Update last error for the next PID cycle
        self.last_error = theta_error
        
        # Use the pre-calculated steering angle as a feedforward term
        feedforward_steering_angle = 0.0 #self.goals[self.goal_idx][3]  # Pre-calculated steering angle

        # Combine feedforward and PID corrections, and apply speed-based smoothing
        steering_adjustment = (feedforward_steering_angle + pid_correction) / (1 + current_speed * 0.5)

        # Limit steering angle to prevent over-steering
        self.drive_steering_angle = max(min(steering_adjustment, 0.2189), -0.2189)
        

        # Speed control: PD control based on pre-calculated speed
        desired_speed = self.goals[self.goal_idx][2]  # Pre-calculated speed from waypoints
        speed_error = desired_speed - current_speed
        
        # PD controller for speed
        kp_speed = 5.0  # Proportional gain for speed
        kd_speed = 0.05  # Derivative gain for speed
        speed_error_derivative = (speed_error - self.previous_trans_err) / 0.05  # Assuming a time step of 0.05
        speed_correction = (kp_speed * speed_error) + (kd_speed * speed_error_derivative)

        # Combine feedforward speed with PD correction
        feedforward_speed = desired_speed
        corrected_speed = feedforward_speed

        # Limit speed to prevent unrealistic values
        max_speed = 3.0 if curvature < 0.2 else 1.5  # Reduce max speed on sharp curves
        corrected_speed = max(min(corrected_speed, max_speed), 1.5)

        # Update previous error for next cycle
        self.previous_trans_err = speed_error

        # Publish drive message
        drive_msg = AckermannDriveStamped()
        drive_msg.header.frame_id = self.base_frame
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.drive.speed = corrected_speed
        drive_msg.drive.acceleration = 0.0
        drive_msg.drive.jerk = 0.0
        drive_msg.drive.steering_angle = self.drive_steering_angle
        drive_msg.drive.steering_angle_velocity = 0.0
        self.drive_pub.publish(drive_msg)
        
        # self.get_logger().info(f'Goal: {self.goals[self.goal_idx]}, Distance: {self.goal_distance}, Pose: {self.car_pose}, Steering: {self.drive_steering_angle}, Speed: {drive_msg.drive.speed}')

        # self.get_logger().info(f'G {self.goals[self.goal_idx]} D {self.goal_distance} P {self.car_pose} S{self.drive_steering_angle} V {drive_msg.drive.speed} ')
        
def main(args=None):
    rclpy.init(args=args)
    goal_publisher_MPC = GoalPublisherMPCSamir()
    rclpy.spin(goal_publisher_MPC)
    goal_publisher_MPC.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()