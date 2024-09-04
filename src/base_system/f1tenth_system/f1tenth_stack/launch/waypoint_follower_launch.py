from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
import yaml

# With this launch file the car drives autonomously from waypoint to waypoint
def generate_launch_description():
    
    # Config for waypoint follower
    wp_follower_config = os.path.join(
        get_package_share_directory('global_planner'), 
        'config', 
        'wp_follower.yaml'
    )
    
    # Waypoint follower for setting the goal
    wp_follower_node = Node(
        package='global_planner',
        executable='wp_follower_node',
        name='wp_follower',
        output='screen',
        parameters=[wp_follower_config]
    )
    
    # Simple velocity controller that drives the car
    velocity_controller_node = Node(
        package='controller',
        executable='velocity_controller', 
        name='velocity_controller',
        output='screen'
    )  
          
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
    
    # Optional: record rosbag 
    # rosbag_cmd = ExecuteProcess(
    #     cmd=['ros2', 'bag', 'record', '-a'],
    # )
    
    ld = LaunchDescription()
    ld.add_action(wp_follower_node)
    ld.add_action(velocity_controller_node)
    ld.add_action(bringup_launch)
    ld.add_action(state_estimation_launch)
    # ld.add_action(rosbag_cmd)
    
    return ld

    
    