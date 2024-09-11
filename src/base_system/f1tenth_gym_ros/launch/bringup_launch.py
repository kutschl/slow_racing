from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
import yaml

# Bringup launch for simulation
def generate_launch_description():
    
    # Rviz config for visualization
    rviz_config = DeclareLaunchArgument(
        'rviz_config',
        description='Pfad zur RViz-Konfigurationsdatei',
        default_value=os.path.join(get_package_share_directory('f1tenth_gym_ros'), 'launch', 'bringup_launch.rviz')
    )
        
    # Config for simulation
    sim_config = os.path.join(get_package_share_directory('f1tenth_gym_ros'),'config','sim.yaml')
    sim_config_dict = yaml.safe_load(open(sim_config, 'r'))
    
    # # Config for amcl
    # amcl_config = os.path.join(
    #     get_package_share_directory('state_estimation'),
    #     'config',
    #     'amcl_sim.yaml'
    # )
    
    # Set to one agent always
    sim_config_dict['bridge']['ros__parameters']['num_agent'] = 1
    
    # Set the map 
    map_name = sim_config_dict['bridge']['map']['name']
    map_path = os.path.join(get_package_share_directory('global_planner'), 'maps', map_name, map_name + '_map')
    map_img_ext = '.png'
    sim_config_dict['bridge']['ros__parameters']['map_path'] = map_path
    sim_config_dict['bridge']['ros__parameters']['map_img_ext'] = map_img_ext
    
    # Set the initial pose 
    wp_config = os.path.join(get_package_share_directory('global_planner'),'config','waypoints.yaml')
    wp_config_dict = yaml.safe_load(open(wp_config, 'r'))
    sx = wp_config_dict['waypoints'][0]['x'] 
    sy = wp_config_dict['waypoints'][0]['y'] 
    stheta = wp_config_dict['waypoints'][0]['theta'] 
    sim_config_dict['bridge']['ros__parameters']['sx'] = sx
    sim_config_dict['bridge']['ros__parameters']['sy'] = sy
    sim_config_dict['bridge']['ros__parameters']['stheta'] = stheta
    
    # Simulation node
    bridge_node = Node(
        package='f1tenth_gym_ros',
        executable='gym_bridge',
        name='gym_bridge',
        parameters=[sim_config_dict['bridge']['ros__parameters']]
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', os.path.join(get_package_share_directory('f1tenth_gym_ros'), 'launch', 'racecar.xacro')])}],
        remappings=[
            ('/robot_description', '/racecar/robot_description')
        ]
    )
    
    # Rviz visualization for simulation
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')]
    )
    
    # Nav2 lifecycle node 
    nav2_lifecycle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[{'use_sim_time': True},
                    {'autostart': True},
                    {'node_names': [
                        'map_server', 
                        'amcl'
                        ]}
                    ]
    )
    
    # Nav2 map server node that publishes map
    nav2_map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[{'yaml_filename': map_path + '.yaml'},
                    {'topic': 'map'},
                    {'frame_id': 'map'},
                    {'output': 'screen'},
                    {'use_sim_time': True}]
    )
    
    # Nav2 amcl for localization
    nav2_amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            # amcl_config,
            {'use_sim_time': True},
            {'initial_pose': {
                'x': sx,
                'y': sy, 
                'yaw': stheta
            }}
        ],
        remappings=[
            ('/amcl_pose', '/racecar/pose')
        ]
    )
    
    # Static tf from map to odom 
    map_to_odom_tf = Node(
        package = "tf2_ros", 
        executable = "static_transform_publisher",
        arguments = ["0", "0", "0", "0", "0", "0", "map", "racecar/odom"]
    )
    
    # Velocity controller
    velocity_controller = Node(
        package='controller',
        executable='velocity_controller',
        name='velocity_controller',
        output='screen'
    )
    
    
    
    ld = LaunchDescription()
    ld.add_action(rviz_config)
    ld.add_action(rviz_node)
    ld.add_action(bridge_node)
    ld.add_action(nav2_lifecycle_node)
    ld.add_action(nav2_map_server_node)
    ld.add_action(robot_state_publisher)
    ld.add_action(velocity_controller)
    ld.add_action(nav2_amcl_node)
    ld.add_action(map_to_odom_tf)
    return ld