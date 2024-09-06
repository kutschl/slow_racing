import rclpy
from rclpy.node import Node
import os 
from ament_index_python.packages import get_package_share_directory
import yaml
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray, Pose, PoseStamped, PoseWithCovarianceStamped
from visualization_msgs.msg import Marker, MarkerArray
# from tf_transformations import quaternion_from_euler
import pandas as pd
import numpy as np

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('wp_follower_node')        
        
        self.declare_parameter('waypoints_file', 'waypoints.yaml')
        self.declare_parameter('robot_pose_topic', '/racecar/pose')
        self.declare_parameter('goal_topic', '/planner/goal')
        self.declare_parameter('waypoints_topic', '/planner/waypoints')
        # self.declare_parameter('centerline_topic', '/centerline')
        
        robot_pose_topic = '/racecar/pose' # TODO
        goal_topic = self.get_parameter('goal_topic').get_parameter_value().string_value
        waypoints_topic = self.get_parameter('waypoints_topic').get_parameter_value().string_value
        # centerline_topic = self.get_parameter('centerline_topic').get_parameter_value().string_value
        
        map_name = 'HRL'
        centerline_path = os.path.join(get_package_share_directory('global_planner'), 'maps', map_name, f'{map_name}_centerline.csv')
        centerline_csv = pd.read_csv(centerline_path)
        centerline = centerline_csv[['x', 'y']].to_numpy()
        self.centerline = centerline[::10]
        
        # # Load waypoints
        # file_path = os.path.join(get_package_share_directory('global_planner'), 'config', 'waypoints.yaml')
        # with open(file_path, 'r') as file:
        #     data = yaml.safe_load(file)
        #     self.waypoints = [(wp['x'], wp['y']) for wp in data['waypoints']]
        #     self.waypoints_theta = [(wp['x'], wp['y'], wp['theta']) for wp in data['waypoints']]
        # self.waypoint_idx = 0
        
        # map config for starting pose 
        map_config_path = os.path.join(get_package_share_directory('global_planner'), 'maps', map_name, f'{map_name}_map.yaml')
        with open(map_config_path, 'r') as file:
            map_config = yaml.safe_load(file)
        self.robot_position = [map_config['starting_pose'][0], map_config['starting_pose'][1]]
        self.goal_idx = 1
        
        self.odom_sub = self.create_subscription(Odometry, '/racecar/odom', self.odom_callback, 10)
        self.goal_pub = self.create_publisher(PoseStamped, goal_topic, 10)
        self.goal_timer = self.create_timer(0.1, self.goal_callback)
        
        # self.robot_pose_sub = self.create_subscription(PoseWithCovarianceStamped, robot_pose_topic, self.pose_callback, 10)
        # self.waypoints_pub = self.create_publisher(PoseArray, waypoints_topic, 10)
        # self.centerline_pub = self.create_publisher(Marker, centerline_topic, 10)
        
        # self.waypoints_timer = self.create_timer(1.0, self.waypoints_callback)
        # self.goal_init = False
        
    def odom_callback(self, msg: Odometry):
        self.robot_position = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        

    # def pose_callback(self, msg: PoseWithCovarianceStamped):
    #     self.current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
    
    
    # def waypoints_callback(self):
    #     pose_array = PoseArray()
    #     pose_array.header.frame_id = 'map'
    #     pose_array.header.stamp = self.get_clock().now().to_msg()
    #     for wp in self.waypoints_theta:
    #         pose = Pose()
    #         q = quaternion_from_euler(0, 0, wp[2])
    #         pose.position.x = wp[0]
    #         pose.position.y = wp[1]
    #         pose.position.z = 0.0
    #         pose.orientation.x = q[0]
    #         pose.orientation.y = q[1]
    #         pose.orientation.z = q[2]
    #         pose.orientation.w = q[3]
    #         pose_array.poses.append(pose)
        
    #     self.waypoints_pub.publish(pose_array)            
        
            
    def goal_callback(self):      
        
        # if hasattr(self, 'current_position') and not self.goal_init:
        #     # suche zu start den waypoint mit min. distanz
        #     d = self.distance_to_waypoint(current_position=self.current_position, waypoint=self.waypoints[self.waypoint_idx])
        #     idx = -1 
        #     for i in range(len(self.waypoints)):
        #         if d > self.distance_to_waypoint(current_position=self.current_position, waypoint=self.waypoints[i]):
        #             idx = i
        #             d = self.distance_to_waypoint(current_position=self.current_position, waypoint=self.waypoints[i])
        #         if idx != -1:
        #             self.waypoint_idx = idx
                
        #     self.goal_init = True
        #     self.get_logger().info("erste wegpunkt init")
            
        # if self.goal_init:
            # check ob nah genug dran, falls ja -> nehme den nächsten wp
        
        # current_waypoint = self.waypoints[self.waypoint_idx]
        
        
        if self.distance_to_waypoint(self.robot_position, self.centerline[self.goal_idx]) < 1.0:
            self.goal_idx += 1
            if self.goal_idx >= len(self.centerline):
                self.goal_idx = 0
       
        self.publish_goal(self.goal_idx)
        
        # # -> suche nächsten wegpunkt als ziel 
        # # -> wenn threshold erreicht dann bitte der nächste wegpunkt!

        # if hasattr(self, 'current_position'):
        #     t1 = 100
        #     t2 = 1.5
            
        #     if self.distance_to_waypoint(current_position=self.current_position, waypoint=self.waypoints[self.waypoint_idx]) > t1:
        #         d = self.distance_to_waypoint(current_position=self.current_position, waypoint=self.waypoints[self.waypoint_idx])
        #         index = -1
        #         for i in range(len(self.waypoints)):
        #             if d > self.distance_to_waypoint(current_position=self.current_position, waypoint=self.waypoints[i]):
        #                 index = i
        #                 d = self.distance_to_waypoint(current_position=self.current_position, waypoint=self.waypoints[i])
        #             if index != -1:
        #                 self.waypoint_idx = index
        
        #     # Get the current waypoint
        #     current_waypoint = self.waypoints[self.waypoint_idx]
            
        #     # Check if the robot is close to the current waypoint
        #     if self.distance_to_waypoint(self.current_position, current_waypoint) < t2:
        #         # Move to the next waypoint
        #         self.waypoint_idx += 1
        #         if self.waypoint_idx >= len(self.waypoints):
        #             self.waypoint_idx = 0
        #             # self.get_logger().info('All waypoints have been reached.')
            
        #     # Publish the next goal
        #     self.publish_goal(self.waypoints[self.waypoint_idx])
        #     self.get_logger().info(f'{self.current_position, self.waypoints[self.waypoint_idx]}')


    # TODO: code cleaning
    def distance_to_waypoint(self, current_position, waypoint):
        d = ((current_position[0] - waypoint[0]*0.05) ** 2 + (current_position[1] - waypoint[1]*0.05) ** 2) ** 0.5
        #self.get_logger().info(str(d))
        return d


    # TODO: code cleaning
    def publish_goal(self, idx):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = self.centerline[idx][0]*0.05
        goal.pose.position.y = self.centerline[idx][1]*0.05
        goal.pose.orientation.w = 1.0  # Face forward
        self.goal_pub.publish(goal)
    
    
def main(args=None):
    rclpy.init(args=args)
    waypoint_follower = WaypointFollower()
    rclpy.spin(waypoint_follower)
    waypoint_follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()