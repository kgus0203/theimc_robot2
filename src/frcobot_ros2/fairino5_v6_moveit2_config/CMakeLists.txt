cmake_minimum_required(VERSION 3.22)
project(fairino5_v6_moveit2_config)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

add_executable(tcp_waypoint_action src/tcp_waypoint_action.cpp)
ament_target_dependencies(tcp_waypoint_action
  rclcpp
  std_msgs
  geometry_msgs
  moveit_ros_planning_interface
)

ament_package()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
install(TARGETS tcp_waypoint_action DESTINATION lib/${PROJECT_NAME})
