from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    
    sim_config_path = os.path.join(get_package_share_directory('f1tenth_gym_ros'),'config','sim.yaml')
    sim_config_dict = yaml.safe_load(open(sim_config_path, 'r'))
    map_name = sim_config_dict['bridge']['ros__parameters']['map_name']
    
    map_config_path = os.path.join(get_package_share_directory('global_planner'),'maps', map_name, f'{map_name}_map.yaml')
    map_config_dict = yaml.safe_load(open(map_config_path, 'r'))
    starting_pose = map_config_dict['starting_pose']
    
    bridge_node = Node(
        package='f1tenth_gym_ros',
        executable='gym_bridge',
        name='gym_bridge',
        parameters=[
            sim_config_dict['bridge']['ros__parameters'],
            {
            'num_agents': 1,
            'map_path': os.path.join(get_package_share_directory('global_planner'), 'maps', map_name, map_name + '_map'),
            'map_img_ext': '.png',
            'sx': starting_pose[0],
            'sy': starting_pose[1],
            'stheta': starting_pose[2]
            }]
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', os.path.join(get_package_share_directory('f1tenth_gym_ros'), 'launch', 'racecar.xacro')])}],
        remappings=[
            ('/robot_description', '/racecar/robot_description')
        ]
    )
    
    rviz_config = DeclareLaunchArgument(
        'rviz_config',
        description='Pfad zur RViz-Konfigurationsdatei',
        default_value=os.path.join(get_package_share_directory('f1tenth_gym_ros'), 'launch', 'planner_launch.rviz')
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')]
    )
    
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
    
    nav2_map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[{'yaml_filename': os.path.join(get_package_share_directory('global_planner'), 'maps', map_name, map_name + '_map') + '.yaml'},
                    {'topic': 'map'},
                    {'frame_id': 'map'},
                    {'output': 'screen'},
                    {'use_sim_time': True}]
    )
    
    amcl_config = os.path.join(
        get_package_share_directory('state_estimation'),
        'config',
        'amcl_sim.yaml'
    )
    
    nav2_amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            amcl_config,
            {'use_sim_time': True},
            {'initial_pose': {
                'x': starting_pose[0],
                'y': starting_pose[1],
                'yaw': starting_pose[2]
            }}
        ],
        remappings=[
            ('/amcl_pose', '/racecar/pose')
        ]
    )
    
    map_to_odom_tf = Node(
        package = "tf2_ros", 
        executable = "static_transform_publisher",
        arguments = ["0", "0", "0", "0", "0", "0", "map", "racecar/odom"]
    )
    
    velocity_controller = Node(
        package='controller',
        executable='velocity_controller',
        name='velocity_controller',
        output='screen'
    )
    
    global_planner_config = os.path.join(
        get_package_share_directory('global_planner'),
        'config',
        'global_planner.yaml'
    )

    centerline_publisher = Node(
        package='global_planner',
        executable='centerline_publisher',
        name='centerline_publisher',
        output='screen',
        parameters=[
            global_planner_config,
            {'map_name': map_name}
            ]
    )
    
    waypoint_follower = Node(
        package='global_planner',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[
            global_planner_config,
            {'map_name': map_name}
            ]
    )

    ld = LaunchDescription()
    ld.add_action(rviz_config)
    ld.add_action(rviz_node)
    ld.add_action(bridge_node)
    ld.add_action(nav2_lifecycle_node)
    ld.add_action(nav2_map_server_node)
    ld.add_action(robot_state_publisher)
    # ld.add_action(velocity_controller)
    ld.add_action(nav2_amcl_node)
    ld.add_action(map_to_odom_tf)
    ld.add_action(centerline_publisher)
    # ld.add_action(waypoint_follower)
    return ld