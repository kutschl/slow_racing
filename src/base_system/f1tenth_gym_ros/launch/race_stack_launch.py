from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
import yaml

# Bringup launch for simulation
def generate_launch_description():
    
    # Rviz config for visualization
    rviz_config = os.path.join(
        get_package_share_directory('f1tenth_gym_ros'), 
        'launch', 
        'race_stack_launch.rviz'
    )
    
    # Include simulation launch
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('f1tenth_gym_ros'), 
                'launch', 
                'bringup_launch.py'
            )
        ),
        launch_arguments={'rviz_config': rviz_config}.items(),
    )
    
    # -----------------------------------------------------------------------------------------
    # GLOBAL PLANNER 
    # -----------------------------------------------------------------------------------------
    # Load sim.yaml
    sim_config = os.path.join(get_package_share_directory('f1tenth_gym_ros'),'config','sim.yaml')
    sim_config_dict = yaml.safe_load(open(sim_config, 'r'))
    
    # Load wp_follower.yaml
    wp_follower_config = os.path.join(get_package_share_directory('global_planner'), 'config', 'wp_follower.yaml')
    wp_follower_config_dict = yaml.safe_load(open(wp_follower_config))
    
    # Set odom_topic for waypoint follower node 
    odom_topic = sim_config_dict['bridge']['ros__parameters']['ego_namespace'] + '/' + sim_config_dict['bridge']['ros__parameters']['ego_odom_topic']
    wp_follower_config_dict['wp_follower_node']['ros__parameters']['odom_topic'] = odom_topic
    
    # Waypoint follower node
    wp_follower_node = Node(
        package='global_planner',
        executable='wp_follower_node',
        name='wp_follower',
        output='screen',
        parameters=[wp_follower_config_dict['wp_follower_node']['ros__parameters']]
    )
    
    # -----------------------------------------------------------------------------------------
    # CONTROLLER: Samir, add you nodes and configs
    # -----------------------------------------------------------------------------------------
    # Simple velocity controller
    velocity_controller_node = Node(
        package='controller',
        executable='velocity_controller', 
        name='velocity_controller',
        output='screen'
    )
    
    ld = LaunchDescription()
    ld.add_action(bringup_launch)
    ld.add_action(wp_follower_node)
    ld.add_action(velocity_controller_node)
    return ld