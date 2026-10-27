"""Run two distinct tools on two distinct PSM arms."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_sensors(context):
    arms = [LaunchConfiguration(f'arm{i}').perform(context) for i in (1, 2)]
    tools = [LaunchConfiguration(f'tool{i}').perform(context) for i in (1, 2)]
    if arms[0] == arms[1]:
        raise ValueError('arm1 and arm2 must be different arms.')
    if tools[0] == tools[1]:
        raise ValueError('tool1 and tool2 must be different tools (one reader per sensor).')
    return [
        Node(
            package='force_n',
            executable='force_n',
            name='force_n',
            namespace=f'/{arm}/shaft',
            parameters=[{'serial_port': f'/dev/{tool}'}],
            output='screen',
        )
        for arm, tool in zip(arms, tools)
    ]


def generate_launch_description():
    arguments = []
    for i, arm, tool in ((1, 'PSM1', 'CLASSIC_LND'), (2, 'PSM2', 'CLASSIC_MBF')):
        arguments.extend([
            DeclareLaunchArgument(
                f'arm{i}', default_value=arm, choices=['PSM1', 'PSM2', 'PSM3'],
                description=f'Arm for sensor {i}; must differ from the other arm.',
            ),
            DeclareLaunchArgument(
                f'tool{i}', default_value=tool,
                choices=['CLASSIC_LND', 'S_LND', 'CLASSIC_MBF', 'S_MBF'],
                description=f'Tool for sensor {i}; must differ from the other tool.',
            ),
        ])
    return LaunchDescription(arguments + [OpaqueFunction(function=_launch_sensors)])
