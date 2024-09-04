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
        
    # Config for another monte carlo localization node
    amcl_config = os.path.join(
        get_package_share_directory('f1tenth_stack'), 
        'config', 
        'amcl.yaml'
    )
    
    # Config for extended kalman filter
    ekf_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'ekf.yaml'
    )
    
    # Config for map parameters
    map_config = os.path.join(
        get_package_share_directory('global_planner'),
        'config',
        'map.yaml'
    )
    
    map_config_dict = yaml.safe_load(open(map_config))
    map_path = os.path.join(get_package_share_directory('global_planner'), 'maps', map_config_dict['map']['map_name'], 'map.yaml')
    use_sim_time = False
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

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
    
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'yaml_filename': map_path},
            {'topic': '/map'},
            {'frame_id': 'map'},
        ],
        remappings=remappings
    )
    
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            amcl_config
        ],
        remappings=remappings
    )
    
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

    
    