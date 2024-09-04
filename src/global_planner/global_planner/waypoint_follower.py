import rclpy
from rclpy.node import Node
import os 
from ament_index_python.packages import get_package_share_directory
import yaml 

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')
        
        self.declare_parameter('map_name', 'Spielberg')
        self.declare_parameter('topic', 'waypoint')
        self.declare_parameter('namespace', 'planner')
        self.declare_parameter('mode', 'centerline')
        
        map_name = self.get_parameter('map_name').get_parameter_value().string_value
        namespace = self.get_parameter('namespace').get_parameter_value().string_value
        topic = f'/{namespace}/{self.get_parameter("topic").get_parameter_value().string_value}'
        mode = self.get_parameter('mode').get_parameter_value().string_value
        
        # TODO: positionsdaten
        # TODO: waypoint publisher and timer 
        
        # TODO: starting pose from map yaml 
        
        # TODO: if mode == centerline: -> load centerline.csv
        # TODO : init waypoint array -> dann nur noch index 
        
        