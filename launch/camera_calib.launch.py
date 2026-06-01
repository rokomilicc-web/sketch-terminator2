import os
import yaml
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

camera_config_path = os.path.join(
        get_package_share_directory('sketch-terminator2'),
        'config',
        'camera_params.yaml'
        )

camera_calib_path = os.path.join(
        get_package_share_directory('sketch-terminator2'),
        'config',
        'camera_calibration_params.yaml'
        )


def generate_launch_description():
    ld = LaunchDescription()

    calib_config_path = os.path.join(
            get_package_share_directory('sketch-terminator2'),
            'config',
            'calib_config.yaml'
            )

    with open(calib_config_path, 'r') as f:
        calib_params = yaml.safe_load(f)['cam2world']['ros__parameters']

    size_arg = f"{calib_params['checkerboard_height']}x{calib_params['checkerboard_width']}"
    square_arg = str(calib_params['square_size'])

    camera_nodes = [
        Node(
            package='usb_cam', executable='usb_cam_node_exe', output='screen',
            name="camera",
            parameters=[camera_config_path]
        )
    ]

    calibration_node = Node(
        package='camera_calibration', executable='cameracalibrator', name='camera_calib', output='screen',
        arguments = ['--size', size_arg, '--square', square_arg],
        remappings=[('image','/image_raw')],
    )

    camera_group = GroupAction(camera_nodes)

    ld.add_action(camera_group)
    ld.add_action(calibration_node)

    return ld