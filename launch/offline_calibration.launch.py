from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
import os
import sys
import yaml
from launch.substitutions import EnvironmentVariable
import pathlib
import launch.actions
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

from launch.actions.execute_process import ExecuteProcess

def generate_launch_description():

    params = os.path.join(get_package_share_directory("hal_allied_vision_camera"), 'params', 'params_offline_calibration.yaml')

    return LaunchDescription([
        Node(
            package='hal_allied_vision_camera',
            executable='offline_camera_calibration',
            name='offline_camera_calibration',
            output={
                    "stdout": "screen",
                    "stderr": "screen",
            },
            parameters=[params],
        ),
])
