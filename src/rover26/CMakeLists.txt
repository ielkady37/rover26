cmake_minimum_required(VERSION 3.8)
project(rover26)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LaneStatus.msg" 
  "msg/ObstacleStatus.msg"
)

ament_export_dependencies(rosidl_default_runtime)

install(DIRECTORY launch urdf meshes config worlds description
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()