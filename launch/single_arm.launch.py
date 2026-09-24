"""Run the Forcen sensor for PSM1."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='force_n',
            executable='force_n',
            name='force_n',
            namespace='/PSM1/shaft',
            parameters=[{'serial_port': '/dev/ttyACM0'}],
            output='screen',
        ),
    ])
