from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    
    # Include bringup launch 
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('f1tenth_stack'), 
                'launch', 
                'bringup_launch.py'
            )
        )
    )
    
    # Include state estimation launch
    state_estimation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('f1tenth_stack'), 
                'launch', 
                'state_estimation_launch.py'
            )
        )
    )
    
    # Goal publisher
    goal_publisher = Node(
        package='global_planner',
        executable='goal_publisher',
        name='goal_publisher',
        output='screen',
    )
    
    # Record rosbag that contains all published topics
    rosbag_cmd = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-a'],
    )
    
    ld = LaunchDescription()
    ld.add_action(bringup_launch)
    ld.add_action(state_estimation_launch)
    ld.add_action(goal_publisher)
    ld.add_action(rosbag_cmd)
    return ld