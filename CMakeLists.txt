cmake_minimum_required(VERSION 3.8)
project(sketch-terminator2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Pronalaženje svih potrebnih ROS 2 paketa
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(yolo_msgs REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(xacro REQUIRED)
find_package(urdf REQUIRED)
find_package(rviz2 REQUIRED)
find_package(ros2_control REQUIRED)
find_package(ros2_controllers REQUIRED)
find_package(dynamixel_hardware REQUIRED)

# 1. INSTALACIJA PYTHON ČVOROVA (Uključujući novi camera_to_world.py)
install(PROGRAMS
  sketch-terminator2/yolo_workspace_processing_node.py
  sketch-terminator2/path_planner_node.py
  sketch-terminator2/move_robot.py
  sketch-terminator2/kinematika.py
  sketch-terminator2/generate_smooth_path.py
  sketch-terminator2/yolo_node.py
  scripts/camera_to_world.py
  DESTINATION lib/${PROJECT_NAME}
)


# 2. INSTALACIJA SVIH RESURSA (Launch datoteke, URDF model, YAML konfiguracije, Meshes)
install(
  DIRECTORY config launch urdf controllers meshes
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
