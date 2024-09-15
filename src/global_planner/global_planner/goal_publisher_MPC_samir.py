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
        self.declare_parameter('odom_topic', '/odometry/filtered')
        
        self.declare_parameter('map_name', 'HRL')
        self.declare_parameter('goal_topic', '/goal')
        self.declare_parameter('namespace', '/planner')
        self.declare_parameter('waypoints_mode', 'centerline')
        # self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('pose_topic', '/amcl_pose')
        self.declare_parameter('drive_topic', '/drive')
        self.declare_parameter('publish_drive', True)
        self.declare_parameter('min_goal_distance', 1.70) # 1.00
        self.declare_parameter('waypoints_step_size', 5) # 20
        self.declare_parameter('use_slam_pose', True)
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('steering_pid_kp', 0.30) # 0.5
        self.declare_parameter('steering_pid_ki', 0.00) # 0.0
        self.declare_parameter('steering_pid_kd', 0.00) # 0.1
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
        self.use_slam_pose = self.get_parameter('use_slam_pose').get_parameter_value().bool_value
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
            # self.get_logger().info(f'centerline waypoints {centerline_xy.shape[0]}, {centerline_xy.shape[1]}')
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
        self.goal_idx = int(map_config_dict['starting_index']/ waypoints_step_size)+3
        self.goal_distance = np.inf
        # self.get_logger().info(f' goal_idx {self.goal_idx}')
        # Init car pose
        self.car_pose = np.array(map_config_dict['starting_pose'])
        
        # Init PID controller for steering
        self.error_sum = 0.0
        self.last_error = 0.0
        self.previous_trans_err = 0.0
        # Subscriber and publisher
        if self.use_slam_pose:
            self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, pose_topic, self.pose_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)
               
        self.goal_pub = self.create_publisher(PoseStamped, goal_topic, 10)
        self.goal_timer = self.create_timer(0.01, self.goal_callback)
        
        if publish_drive:
            self.drive_pub = self.create_publisher(AckermannDriveStamped, drive_topic, 10)
            self.drive_timer = self.create_timer(0.02, self.drive_callback)   
            
        self.drive_msg = AckermannDriveStamped()
        self.drive_msg.header.frame_id = self.base_frame
        self.drive_msg.header.stamp = self.get_clock().now().to_msg()
        self.drive_msg.drive.speed = 0.0
        self.drive_msg.drive.acceleration = 0.0
        self.drive_msg.drive.jerk = 0.0
        self.drive_msg.drive.steering_angle = 0.0
        self.drive_msg.drive.steering_angle_velocity = 0.0
        self.drive_pub.publish(self.drive_msg)
        
    def pose_callback(self, msg: PoseWithCovarianceStamped):
        # update car position from pose
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        _,_,theta = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        self.car_pose = np.array([x,y,theta])

    def odom_callback(self, msg: Odometry):
        # update car position from odom
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        _,_,theta = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        
        if not self.use_slam_pose:
            self.car_pose = np.array([x,y,theta])
        
        self.racecar_twist = [msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.angular.z]

    def goal_callback(self):
        # update distance between car and goal
        self.goal_distance = np.linalg.norm((self.car_pose[0:2]-self.goals[self.goal_idx][0:2]))
        # check goal
        if self.goal_distance < self.min_goal_distance:
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


    def drive_callback(self):       
        dx = self.goals[self.goal_idx ][0] - self.car_pose[0]
        dy = self.goals[self.goal_idx ][1] - self.car_pose[1]
        theta_error = math.atan2(dy, dx) - self.car_pose[2]
        theta_error = (theta_error + math.pi) % (2 * math.pi) - math.pi

        # Adjust PID kp based on steering angle and speed
        """this is actually the theta error to correct the steering maybe i can do this different""" 
        recommended_steering_angle = abs(theta_error) # abs(self.goals[self.goal_idx - 2][3])  # Use absolute value of the steering angle
        recommended_speed = self.racecar_twist[0] # self.goals[self.goal_idx - 3][2]
            
        #Define kp min and max values
        kp_min = 0.005  # Minimum kp for straight sections
        kp_max = 0.30  # Maximum kp for sharp corners
        # Steering thresholds and speed thresholds
        max_steering_angle = 0.25  # Maximum steering angle to consider (beyond this is tight corner)
        min_steering_angle = 0.05  # Minimum steering angle for straight driving
        max_speed = 5.0  # Max speed (straight sections)
        min_speed = 1.5  # Min speed (tight corners)

        # Scale kp dynamically based on steering angle and speed
        # Calculate a dynamic factor for the steering angle
        steering_factor = (recommended_steering_angle - min_steering_angle) / (max_steering_angle - min_steering_angle)
        steering_factor = np.clip(steering_factor, 0, 2)  # Clamp between 0 and 1

        # Calculate a dynamic factor for speed
        speed_factor = (recommended_speed - min_speed) / (max_speed - min_speed)
        speed_factor = np.clip(speed_factor, 0, 1)  # Clamp between 0 and 1
        # Invert speed factor so that lower speed increases #kp (corners) and higher speed decreases kp (straights)
        speed_factor = 1 - speed_factor
        
        # Weighted combination of steering and speed factors
        # steering_weight = 0.5       # Give 70% weight to the steering factor
        # speed_weight = 0.5          # Give 30% weight to the speed factor

        # combined_factor = (steering_weight * steering_factor) + (speed_weight * speed_factor)

        combined_factor =  steering_factor # speed_factor
        # Calculate dynamic kp using the combined factor
        dynamic_kp = kp_min + (kp_max - kp_min) * combined_factor

        # Apply the adjusted kp to the PID controller
        self.steering_pid_kp = dynamic_kp

        
        # PID control for steering
        self.error_sum += theta_error
        delta_error = theta_error - self.last_error
        steering_angle = (self.steering_pid_kp * theta_error) + \
                        (self.steering_pid_ki * self.error_sum) + \
                        (self.steering_pid_kd * delta_error)
        self.last_error = theta_error
            
        # load mpc values 
        self.get_logger().info(f'steering_factor{steering_factor}, speed_factor{speed_factor}, \
                               recommend{recommended_steering_angle} \
                               kp {dynamic_kp} speednow {self.racecar_twist[0]}, wantV {recommended_speed},\
                               ')
                               #T {theta_error} G {self.goals[self.goal_idx]} D {self.goal_distance} P {self.car_pose} S{steering_angle}  
        
        speed = min(self.goals[self.goal_idx][2], 4.00)
        
        if speed > 2.80:
            speed = speed + 0.10 
        # publish drive
        #drive_msg = AckermannDriveStamped()
        self.drive_msg.header.frame_id = self.base_frame
        self.drive_msg.header.stamp = self.get_clock().now().to_msg()
        self.drive_msg.drive.speed =  speed
        self.drive_msg.drive.acceleration = 0.0
        self.drive_msg.drive.jerk = 0.0
        self.drive_msg.drive.steering_angle = steering_angle
        self.drive_msg.drive.steering_angle_velocity = 0.0
        self.drive_pub.publish(self.drive_msg)
                
def main(args=None):
    rclpy.init(args=args)
    goal_publisher_MPC = GoalPublisherMPCSamir()
    rclpy.spin(goal_publisher_MPC)
    goal_publisher_MPC.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()