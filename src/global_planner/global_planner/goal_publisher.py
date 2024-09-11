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

class GoalPublisher(Node):
    def __init__(self):
        super().__init__('goal_publisher')
        
        # Parameter
        self.declare_parameter('map_name', 'HRL')
        self.declare_parameter('goal_topic', '/goal')
        self.declare_parameter('namespace', '/planner')
        self.declare_parameter('waypoints_mode', 'centerline')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('pose_topic', '/amcl_pose')
        self.declare_parameter('drive_topic', '/drive')
        self.declare_parameter('publish_drive', True)
        self.declare_parameter('min_goal_distance', 1.00)
        self.declare_parameter('waypoints_step_size', 20) # TODO: REMOVE
        self.declare_parameter('use_slam_pose', True)
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('steering_pid_kp', 0.47) # 0.5
        self.declare_parameter('steering_pid_ki', 0.00) # 0.0
        self.declare_parameter('steering_pid_kd', 0.13) # 0.1
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
        waypoints_step_size = self.get_parameter('waypoints_step_size').get_parameter_value().integer_value # TODO: REMOVE
        self.min_goal_distance = self.get_parameter('min_goal_distance').get_parameter_value().double_value
        use_slam_pose = self.get_parameter('use_slam_pose').get_parameter_value().bool_value
        publish_drive = self.get_parameter('publish_drive').get_parameter_value().bool_value
        self.steering_pid_kp = self.get_parameter('steering_pid_kp').get_parameter_value().double_value
        self.steering_pid_ki = self.get_parameter('steering_pid_ki').get_parameter_value().double_value
        self.steering_pid_kd = self.get_parameter('steering_pid_kd').get_parameter_value().double_value
        self.drive_speed = self.get_parameter('drive_speed').get_parameter_value().double_value
        self.drive_steering_angle = 0.0
        
        # Load map config
        map_config_path = os.path.join(get_package_share_directory('global_planner'), 'maps', map_name, f'{map_name}_map.yaml')
        with open(map_config_path, 'r') as file:
            map_config_dict = yaml.safe_load(file)
                
        # Waypoints: centerline (option 1)
        if waypoints_mode == 'centerline':
            # load centerline
            centerline_path = os.path.join(get_package_share_directory('global_planner'), 'maps', map_name, f'{map_name}_centerline.csv')
            centerline_csv = pd.read_csv(centerline_path)
            centerline_xy = centerline_csv[['x', 'y']].to_numpy()
            centerline_xy = centerline_xy[::waypoints_step_size] # TODO: REMOVE
            centerline_xy = centerline_xy*float(map_config_dict['resolution'])
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
        
        # Subscriber and publisher
        if use_slam_pose: 
            self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, pose_topic, self.pose_callback, 10)
        else:
            self.odom_sub = self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)
               
        self.goal_pub = self.create_publisher(PoseStamped, goal_topic, 10)
        self.goal_timer = self.create_timer(0.1, self.goal_callback)
        
        if publish_drive:
            self.drive_pub = self.create_publisher(AckermannDriveStamped, drive_topic, 10)
            self.drive_timer = self.create_timer(0.1, self.drive_callback)   
        
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
        self.car_pose = np.array([x,y,theta])
        
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
        _, _, z, w =  quaternion_from_euler(ai=0.0, aj=0.0, ak=self.goals[self.goal_idx][2])
        goal_msg.pose.orientation.w = w
        goal_msg.pose.orientation.z = z
        self.goal_pub.publish(goal_msg)
        
        
    def drive_callback(self):
        # steering PID controller 
        dx = self.goals[self.goal_idx][0]-self.car_pose[0]
        dy = self.goals[self.goal_idx][1]-self.car_pose[1]
        theta_error = math.atan2(dy,dx) - self.car_pose[2]
        theta_error = (theta_error + math.pi) % (2 * math.pi)  - math.pi
        self.last_error = theta_error
        self.error_sum += theta_error
        delta_error = theta_error - self.last_error
        self.drive_steering_angle = (self.steering_pid_kp * theta_error) + (self.steering_pid_ki * self.error_sum) + (self.steering_pid_kd * delta_error)
        self.last_error = theta_error
                  
        # publish drive
        drive_msg = AckermannDriveStamped()
        drive_msg.header.frame_id = self.base_frame
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.drive.speed = self.drive_speed
        drive_msg.drive.acceleration = 0.0
        drive_msg.drive.jerk = 0.0
        drive_msg.drive.steering_angle = self.drive_steering_angle
        drive_msg.drive.steering_angle_velocity = 0.0
        self.drive_pub.publish(drive_msg)
        self.get_logger().info(f'G {self.goals[self.goal_idx]} D {self.goal_distance} P {self.car_pose} S{self.drive_steering_angle} V {self.drive_speed} ')
        
def main(args=None):
    rclpy.init(args=args)
    goal_publisher = GoalPublisher()
    rclpy.spin(goal_publisher)
    goal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()