import os
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

    ## use use args if you want to
    arg_size = DeclareLaunchArgument(
            'size',
            default_value='7x9',
            description='Checkerboard size'
        )
    arg_square = DeclareLaunchArgument(
            'square',
            default_value='0.0175',
            description='Square size'
        )

    camera_nodes = [
        Node(
            package='usb_cam', executable='usb_cam_node_exe', output='screen',
            name="camera",
            parameters=[camera_config_path]
        )
    ]

    calibration_node = Node(
        package='camera_calibration', executable='cameracalibrator', name='camera_calib', output='screen',
        arguments = ['--size', '7x9', '--square', '0.0175'], ## make sure these are correct
        remappings=[('image','/image_raw')],
    )

    camera_group = GroupAction(camera_nodes)

    ld.add_action(camera_group)
    ld.add_action(calibration_node)

    return ld