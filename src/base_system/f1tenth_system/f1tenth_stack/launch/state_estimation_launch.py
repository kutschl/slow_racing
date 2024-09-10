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
    
    # # Optional: Include rosbag teleop launch that the car can drive teleop and record rosbag
    # rosbag_teleop_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(
    #             get_package_share_directory('f1tenth_stack'), 
    #             'launch', 
    #             'rosbag_teleop_launch.py'
    #         )
    #     )
    # )
    
    # General parameters
    use_sim_time = False
    
    # Load global planner config
    global_planner_config_path = os.path.join(get_package_share_directory('global_planner'), 'config', 'global_planner.yaml')
    global_planner_config_dict = yaml.safe_load(open(global_planner_config_path, 'r'))
    
    # Set map name for map server
    map_name = global_planner_config_dict['/**']['ros__parameters']['map_name']
    
    # Set starting pose for amcl
    map_config_path = os.path.join(get_package_share_directory('global_planner'), 'maps', map_name, f'{map_name}_map.yaml')    
    map_config_dict = yaml.safe_load(open(map_config_path, 'r'))
    starting_pose = map_config_dict['starting_pose']
    amcl_config_path = os.path.join(get_package_share_directory('f1tenth_stack'), 'config', 'amcl.yaml')
    amcl_config_dict = yaml.safe_load(open(amcl_config_path, 'r'))
    amcl_config_dict['amcl']['ros__parameters']['initial_pose']['x'] = starting_pose[0]
    amcl_config_dict['amcl']['ros__parameters']['initial_pose']['y'] = starting_pose[1]
    amcl_config_dict['amcl']['ros__parameters']['initial_pose']['z'] = 0.0
    amcl_config_dict['amcl']['ros__parameters']['initial_pose']['yaw'] = starting_pose[2]
    amcl_config_dict['amcl']['ros__parameters']['use_sim_time'] = False
    
    # Config for extended kalman filter
    ekf_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'ekf.yaml'
    )
    
    # Nav lifecycle node: necessary for map server and amcl
    nav_lifecycle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'node_names': [
                'map_server', 
                'amcl'
            ]}
        ]
    )
    
    # Map server for publishing the map
    map_server_remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'yaml_filename': map_config_path},
            {'topic': '/map'},
            {'frame_id': 'map'},
        ],
        remappings=map_server_remappings
    )
    
    # AMCL node for localization
    amcl_remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[amcl_config_dict['amcl']['ros__parameters']],
        remappings=amcl_remappings
    )
    
    # EKF node for fusing IMU and odometry data for better amcl input
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config],
    )
    
    ld = LaunchDescription()
    # ld.add_action(rosbag_teleop_launch)
    ld.add_action(nav_lifecycle_node)
    ld.add_action(map_server_node)
    ld.add_action(amcl_node)
    ld.add_action(ekf_node)
    
    return ld

    
    