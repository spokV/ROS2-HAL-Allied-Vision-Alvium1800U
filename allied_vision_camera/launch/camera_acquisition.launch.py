import os
import yaml

from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    params_allied_filepath = os.path.join(
        get_package_share_directory("hal_allied_vision_camera"),
        "params",
        "params.yaml",
    )

    params_acquisition_filepath = os.path.join(
        get_package_share_directory("hal_allied_vision_camera"),
        "params",
        "params_camera_acquisition.yaml",
    )

    # with open(configFilepath, 'r') as file:
    #     configParams = yaml.safe_load(file)['stereo_acquisition_node']['ros__parameters']

    node_cam = Node(
        package="hal_allied_vision_camera",
        executable="av_node",
        name="av_node",
        parameters=[params_allied_filepath],
    )

    cam_acquisition_node = Node(
        package="hal_allied_vision_camera",
        executable="camera_acquisition",
        name="camera_acquisition",
        parameters=[params_acquisition_filepath],
    )

    return LaunchDescription([node_cam, cam_acquisition_node])
