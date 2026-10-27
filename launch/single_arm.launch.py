"""Run one tool's Forcen sensor on a selected PSM arm."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'arm', default_value='PSM1', choices=['PSM1', 'PSM2', 'PSM3'],
            description='Arm receiving the sensor topics.',
        ),
        DeclareLaunchArgument(
            'tool', default_value='CLASSIC_LND',
            choices=['CLASSIC_LND', 'S_LND', 'CLASSIC_MBF', 'S_MBF'],
            description='Tool alias from the installed udev rules.',
        ),
        Node(
            package='force_n',
            executable='force_n',
            name='force_n',
            namespace=['/', LaunchConfiguration('arm'), '/shaft'],
            parameters=[{'serial_port': ['/dev/', LaunchConfiguration('tool')]}],
            output='screen',
        ),
    ])
