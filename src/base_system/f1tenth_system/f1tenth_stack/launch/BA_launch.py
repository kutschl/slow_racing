from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
import yaml

# Launch file for testing the state estimation module
def generate_launch_description():
    
    # Optional: Include rosbag teleop launch that the car can drive teleop and record rosbag
    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('f1tenth_stack'), 
                'launch', 
                'teleop_launch.py'
            )
        )
    )
    
    # General parameters
    use_sim_time = False
    
    # Config for extended kalman filter
    ekf_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'ekf.yaml'
    )
    
    # EKF node for fusing IMU and odometry data for better amcl input
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config],
    )
    
    # Record rosbag that contains all published topics
    rosbag_cmd = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-a'],
    )
    
    ld = LaunchDescription()
    # ld.add_action(rosbag_teleop_launch)
    # ld.add_action(nav_lifecycle_node)
    # ld.add_action(map_server_node)
    # ld.add_action(amcl_node)
    ld.add_action(ekf_node)
    ld.add_action(teleop_launch)
    ld.add_action(rosbag_cmd)
    
    return ld

    
    