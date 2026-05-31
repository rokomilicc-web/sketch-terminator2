import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    package_name = "sketch-terminator2"
    robot_name = "prarob_manipulator"
    
    # Putanje do konfiguracijskih datoteka unutar share direktorija paketa
    rviz_config = os.path.join(get_package_share_directory(package_name), "launch", "prarob_manipulator.rviz")
    robot_description = os.path.join(get_package_share_directory(package_name), "urdf", robot_name + ".urdf.xacro")
    robot_description_config = xacro.process_file(robot_description)
    
    controller_config = os.path.join(get_package_share_directory(package_name), "controllers", "controllers.yaml")
    vision_config_file = os.path.join(get_package_share_directory(package_name), "config", "vision_config.yaml")
    camera_params_file = os.path.join(get_package_share_directory(package_name), "config", "camera_params.yaml")

    prefix_cmd = None
    if os.system("which gnome-terminal > /dev/null 2>&1") == 0:
        prefix_cmd = "gnome-terminal --"
    elif os.system("which xterm > /dev/null 2>&1") == 0:
        prefix_cmd = "xterm -e"

    return LaunchDescription([
        # 1. ROS2 HARDVERSKO SUČELJE I KONTROLER MANAGERI
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[{"robot_description": robot_description_config.toxml()}, controller_config],
            output="screen",
        ),

        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
            output="screen",
        ),

        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["velocity_controller", "-c", "/controller_manager"],
            output="screen",
        ),

        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
            output="screen",
        ),

        # 2. VIZUALIZACIJA I MODEL ROBOTA
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[{"robot_description": robot_description_config.toxml()}],
            output="screen",
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            output="screen",
        ),

        # 2.5 CAMERA NODE
        Node(
            package='usb_cam', 
            executable='usb_cam_node_exe', 
            name="camera",
            output='screen',
            parameters=[camera_params_file]
        ),

        # 3. VIZIJSKI ČVOR (YOLO) S UČITANIM PARAMETRIMA KALIBRACIJE
        Node(
            package=package_name,
            executable='yolo_workspace_processing_node.py',
            name='yolo_workspace_processing_node',
            output='screen',
            parameters=[vision_config_file]
        ),

        # 3.1. YOLO DETECTION ČVOR (Pokreće YOLOv12 inferenciju na slici s kamere)
        Node(
            package=package_name,
            executable='yolo_node.py',
            name='yolo_node',
            output='screen',
            parameters=[{
                'model': 'yolo12m.pt',
                'device': 'cuda:0',
                'threshold': 0.5,
                'iou': 0.5,
                'input_image_topic': '/image_raw',
                'detections_topic': '/yolo/detections',
                'debug_image_topic': 'dbg_image',
                'use_debug': True
            }]
        ),

        # 4. ČVOR ZA PLANIRANJE PUTANJE
        Node(
            package=package_name,
            executable='path_planner_node.py',
            name='path_planner_node',
            output='screen'
        ),

        # 5. ČVOR DIREKTNE KINEMATIKE (Čita /joint_states -> daje /marker_end_point)
        Node(
            package=package_name,
            executable="kinematika.py",
            name="kinematika_node",
            output="screen",
        ),

        # 6. ČVOR GENERATORA GLATKE PUTANJE - OTVARA SE U POSEBNOM TERMINALU
        Node(
            package=package_name,
            executable="generate_smooth_path.py",
            name="smooth_path_generator",
            output="screen",
            prefix=prefix_cmd
        ),

        # 7. ČVOR ZA PROSLJEĐIVANJE PUTANJE MOTORIMA - OTVARA SE U POSEBNOM TERMINALU
        Node(
            package=package_name,
            executable="move_robot.py",
            name="move_robot_node",
            output="screen",
            prefix=prefix_cmd
        )
    ])