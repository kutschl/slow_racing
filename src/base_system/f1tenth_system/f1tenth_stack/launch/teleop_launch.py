from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

# This launch controls the real car teleop with a joystick.
def generate_launch_description():

    # Config for teleop control
    joy_teleop_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'joy_teleop.yaml'
    )
    
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy',
        parameters=[joy_teleop_config]
    )
    
    joy_teleop_node = Node(
        package='joy_teleop',
        executable='joy_teleop',
        name='joy_teleop',
        parameters=[joy_teleop_config]
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
    
    ld = LaunchDescription()
    ld.add_action(joy_node)
    ld.add_action(joy_teleop_node)
    ld.add_action(bringup_launch)

    return ld
