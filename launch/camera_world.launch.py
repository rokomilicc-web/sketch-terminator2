import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription  # noqa: E402
from launch.actions import GroupAction  # noqa: E402
from launch_ros.actions import Node  # noqa: E402
from launch.actions import DeclareLaunchArgument

camera_config_path = os.path.join(
        get_package_share_directory('sketch-terminator2'),
        'config',
        'camera_params.yaml'
        )


def generate_launch_description():
    ld = LaunchDescription()

    camera_nodes = [
        Node(
            package='usb_cam', executable='usb_cam_node_exe', output='screen',
            name="camera",
            parameters=[camera_config_path]
        )
    ]

    cam_to_world = Node(
        package='sketch-terminator2', executable='camera_to_world.py', name='cam2world', output='screen'
    )

    camera_group = GroupAction(camera_nodes)

    ld.add_action(camera_group)
    ld.add_action(cam_to_world)
    return ld
