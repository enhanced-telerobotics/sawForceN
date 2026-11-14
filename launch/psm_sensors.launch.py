"""Automatically match registered ForceN tools to the PSM reporting them."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', choices=['true', 'false']),
        Node(
            package='force_n', executable='force_n_tool_manager', name='force_n_tool_manager',
            parameters=[{'gui': ParameterValue(LaunchConfiguration('gui'), value_type=bool)}],
            output='screen',
        ),
    ])
