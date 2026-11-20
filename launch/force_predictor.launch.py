"""Predict force for the active ForceN tool on each dVRK PSM."""

from pathlib import Path
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from saw_force_n.predict import model_directories
from saw_force_n.tools import ARMS


def start_predictors(context):
    arm = LaunchConfiguration('arm').perform(context)
    arms = ARMS if arm == 'all' else (arm,)
    return [
        ExecuteProcess(
            cmd=[LaunchConfiguration('predictor_python'), '-m', 'saw_force_n.predict',
                 '--arm', psm, '--model-directory', LaunchConfiguration('model_directory')],
            name=f'force_predictor_{psm}',
            output='screen',
        )
        for psm in arms
    ]


def generate_launch_description():
    local_python = Path.home() / 'Documents/envs/torch_stable/bin/python'
    return LaunchDescription([
        DeclareLaunchArgument('arm', default_value='all', choices=['all', *ARMS],
                              description='Watch all PSMs or limit prediction to one arm.'),
        DeclareLaunchArgument('model_directory', default_value=str(model_directories('')[0]),
                              description='Checkpoint directory. The default also searches ROS_HOME.'),
        DeclareLaunchArgument('predictor_python',
                              default_value=str(local_python) if local_python.is_file() else sys.executable,
                              description='Python interpreter with PyTorch and access to ROS 2.'),
        OpaqueFunction(function=start_predictors),
    ])
