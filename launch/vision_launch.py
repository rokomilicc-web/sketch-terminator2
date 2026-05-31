import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Dohvaćanje instalacijskog direktorija tvog NOVOG paketa
    package_dir = get_package_share_directory('sketch-terminator2')
    
    # 2. Definiranje točne putanje do vision_config.yaml datoteke
    config_file_path = os.path.join(package_dir, 'config', 'vision_config.yaml')

    # 3. Definiranje vizijskog čvora
    yolo_processing_node = Node(
        package='sketch-terminator2',
        executable='yolo_workspace_processing_node.py',  # Pripazi na ekstenziju ovisno o CMakeLists
        name='yolo_workspace_processing_node',
        output='screen',
        parameters=[config_file_path]  # Učitavanje kalibracijskih parametara
    )

    # 4. Definiranje čvora za planiranje putanje
    path_planner_node = Node(
        package='sketch-terminator2',
        executable='path_planner_node.py',  # Pripazi na ekstenziju ovisno o CMakeLists
        name='path_planner_node',
        output='screen'
        # Ako planer zatreba specifične parametre iz YAML-a, možeš i njemu dodati: parameters=[config_file_path]
    )

    # 5. Vraćanje opisa launcha sustavu (pokreću se oba čvora)
    return LaunchDescription([
        yolo_processing_node,
        path_planner_node
    ])