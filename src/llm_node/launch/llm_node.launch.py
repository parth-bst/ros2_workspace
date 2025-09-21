#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'openai_api_key',
            default_value='',
            description='OpenAI API key (can also be set via environment variable)'
        ),
        
        Node(
            package='llm_node',
            executable='llm_node',
            name='llm_node',
            output='screen',
            parameters=[{
                'openai_api_key': LaunchConfiguration('openai_api_key')
            }],
            env=[{
                'OPENAI_API_KEY': LaunchConfiguration('openai_api_key')
            }]
        )
    ])
