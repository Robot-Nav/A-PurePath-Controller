import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('graceful_controller_ros2')
    
    controller_params_file = os.path.join(pkg_share, 'config', 'controller_params.yaml')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )
    
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=controller_params_file,
        description='Path to the controller parameters file'
    )
    
    controller_node = Node(
        package='graceful_controller_ros2',
        executable='graceful_controller_ros2_node',
        name='graceful_controller',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        declare_params_file,
        controller_node,
    ])
