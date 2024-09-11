from sys import prefix
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
import yaml
from pathlib import Path


def generate_launch_description():
    
    # General parameter
    home_path = str(Path.home())
    
    # Load sim parameter 
    sim_config_path = os.path.join(get_package_share_directory('f1tenth_gym_ros'),'config','sim.yaml')
    sim_config_dict = yaml.safe_load(open(sim_config_path, 'r'))
    namespace = sim_config_dict['bridge']['ros__parameters']['ego_namespace']
    odom_topic = f"/{namespace}/{sim_config_dict['bridge']['ros__parameters']['ego_odom_topic']}"
    scan_topic = f"/{namespace}/{sim_config_dict['bridge']['ros__parameters']['ego_scan_topic']}"
    drive_topic = f"/{namespace}/{sim_config_dict['bridge']['ros__parameters']['ego_drive_topic']}"
    pose_topic = f"/{namespace}/pose"
    map_frame = 'map'
    base_frame = f"/{namespace}/base_link"
    map_name = sim_config_dict['bridge']['ros__parameters']['map_name']
    
    # Change global planner parameter for sim 
    global_planner_config_path = os.path.join(get_package_share_directory('global_planner'),'config','global_planner.yaml')
    global_planner_config_dict = yaml.safe_load(open(global_planner_config_path, 'r'))
    global_planner_config_dict['/**']['ros__parameters']['map_name'] = map_name
    global_planner_config_dict['/**']['ros__parameters']['odom_topic'] = odom_topic
    global_planner_config_dict['/**']['ros__parameters']['pose_topic'] = pose_topic
    global_planner_config_dict['/**']['ros__parameters']['drive_topic'] = drive_topic
    global_planner_config_dict['/**']['ros__parameters']['base_frame'] = base_frame
    global_planner_config_dict['/**']['ros__parameters']['map_frame'] = map_frame
    global_planner_config_dict['goal_publisher']['ros__parameters']['use_slam_pose'] = False
    global_planner_config_dict['goal_publisher']['ros__parameters']['publish_drive'] = True
    
    # Load starting pose for racecar
    map_config_path = os.path.join(get_package_share_directory('global_planner'),'maps', map_name, f'{map_name}_map.yaml')
    map_config_dict = yaml.safe_load(open(map_config_path, 'r'))
    starting_pose = map_config_dict['starting_pose']
    
    # Add parameter for sim 
    sim_config_dict['bridge']['ros__parameters']['num_agents'] = 1
    sim_config_dict['bridge']['ros__parameters']['map_path'] = os.path.join(get_package_share_directory('global_planner'), 'maps', map_name, map_name + '_map')
    sim_config_dict['bridge']['ros__parameters']['map_img_ext'] = '.png'
    sim_config_dict['bridge']['ros__parameters']['sx'] = starting_pose[0]
    sim_config_dict['bridge']['ros__parameters']['sy'] = starting_pose[1]
    sim_config_dict['bridge']['ros__parameters']['stheta'] = starting_pose[2] 
    


    bridge_node = Node(
        package='f1tenth_gym_ros',
        executable='gym_bridge',
        name='gym_bridge',
        parameters=[sim_config_dict['bridge']['ros__parameters']],
        prefix=[os.path.join(home_path, '.virtualenvs/gym_env/bin/python3')]
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
                        # 'amcl'
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
    
    # amcl_config = os.path.join(
    #     get_package_share_directory('state_estimation'),
    #     'config',
    #     'amcl_sim.yaml'
    # )
    
    nav2_amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            # amcl_config,
            {'use_sim_time': True},
            {'initial_pose': {
                'x': starting_pose[0],
                'y': starting_pose[1],
                'yaw': starting_pose[2]
            }}
        ],
        remappings=[
            ('/amcl_pose', pose_topic)
        ]
    )
    
    map_to_odom_tf = Node(
        package = "tf2_ros", 
        executable = "static_transform_publisher",
        arguments = ["0", "0", "0", "0", "0", "0", "map", "racecar/odom"]
    )
    
    centerline_publisher = Node(
        package='global_planner',
        executable='centerline_publisher',
        name='centerline_publisher',
        output='screen',
        parameters=[
            global_planner_config_dict['/**']['ros__parameters']
        ]
    )
    
    goal_publisher = Node(
        package='global_planner',
        executable='goal_publisher',
        name='goal_publisher',
        output='screen',
        parameters=[
            global_planner_config_dict['/**']['ros__parameters'],
            global_planner_config_dict['goal_publisher']['ros__parameters']
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
    # ld.add_action(nav2_amcl_node)
    ld.add_action(map_to_odom_tf)
    ld.add_action(centerline_publisher)
    ld.add_action(goal_publisher)
    return ld