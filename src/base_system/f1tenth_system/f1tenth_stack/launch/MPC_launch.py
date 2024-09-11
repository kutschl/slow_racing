from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
import yaml
from pathlib import Path

def generate_launch_description():
    
    home = str(Path.home())
    
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
    
    MPC_controller = Node(
        package='controller',
        executable='MPC_controller',
        name='MPC_controller',
        output='screen',
        prefix=[os.path.join(home, '.virtualenvs/mpc_env/bin/python3')]
    )
    
    # Record rosbag that contains all published topics
    rosbag_cmd = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-a'],
    )
    
    ld = LaunchDescription()
    ld.add_action(bringup_launch)
    ld.add_action(state_estimation_launch)
    ld.add_action(rosbag_cmd)
    ld.add_action(MPC_controller)
    return ld