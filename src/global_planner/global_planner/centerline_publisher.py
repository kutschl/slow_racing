import rclpy
from rclpy.node import Node
import os 
from ament_index_python.packages import get_package_share_directory
import pandas as pd 
import numpy as np 
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import yaml

class CenterlinePublisher(Node):
    def __init__(self):
        super().__init__('centerline_publisher')
        
        self.declare_parameter('map_name', 'Spielberg')
        self.declare_parameter('topic', '/centerline')
        self.declare_parameter('namespace', '/planner')
        
        map_name = self.get_parameter('map_name').get_parameter_value().string_value
        namespace = self.get_parameter('namespace').get_parameter_value().string_value
        topic = f'{namespace}{self.get_parameter("topic").get_parameter_value().string_value}'

        # open csv
        centerline_csv_path = os.path.join(get_package_share_directory('global_planner'), 'maps', map_name, f'{map_name}_centerline.csv')
        centerline_csv = pd.read_csv(centerline_csv_path)
                
        # open map configs
        map_configs_path = os.path.join(get_package_share_directory('global_planner'), 'maps', map_name, f'{map_name}_map.yaml')
        map_configs = yaml.safe_load(open(map_configs_path, 'r'))
        map_resolution = map_configs['resolution']

        # centerline publisher
        self.centerline_pub = self.create_publisher(Marker, topic, 10)
        self.centerline_timer = self.create_timer(1.0, self.centerline_callback)
        
        # centerline message
        self.centerline_msg = Marker()
        self.centerline_msg.header.frame_id = 'map'
        self.centerline_msg.header.stamp = self.get_clock().now().to_msg()
        self.centerline_msg.id = 0
        self.centerline_msg.type = Marker.POINTS
        self.centerline_msg.action = Marker.ADD
        self.centerline_msg.scale.x = 0.1
        self.centerline_msg.scale.y = 0.1
        self.centerline_msg.color.r = 0.0
        self.centerline_msg.color.g = 1.0
        self.centerline_msg.color.b = 0.0
        self.centerline_msg.color.a = 1.0
        for idx in range(len(centerline_csv.index)):
            point_msg = Point()
            point_msg.x = centerline_csv.loc[idx, 'x']*map_resolution
            point_msg.y = centerline_csv.loc[idx, 'y']*map_resolution
            self.centerline_msg.points.append(point_msg)
        
    def centerline_callback(self):
        self.centerline_pub.publish(self.centerline_msg)     
    
def main(args=None):
    rclpy.init(args=args)
    centerline_publisher = CenterlinePublisher()
    rclpy.spin(centerline_publisher)
    centerline_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()    