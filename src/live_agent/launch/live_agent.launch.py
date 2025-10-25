#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('live_agent')
    
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_dir, 'config', 'agent.yaml'),
        description='Path to agent configuration file'
    )
    
    topics_file_arg = DeclareLaunchArgument(
        'topics_file',
        default_value=os.path.join(pkg_dir, 'config', 'topics.yaml'),
        description='Path to topics configuration file'
    )
    
    # Live Agent Node
    live_agent_node = Node(
        package='live_agent',
        executable='live_agent',
        name='live_agent',
        output='screen',
        parameters=[
            {'config_file': LaunchConfiguration('config_file')},
            {'topics_file': LaunchConfiguration('topics_file')},
        ],
        remappings=[
            ('/live_agent/user_text', '/user_input'),
            ('/live_agent/response', '/agent_response'),
        ]
    )
    
    return LaunchDescription([
        config_file_arg,
        topics_file_arg,
        live_agent_node,
    ])
