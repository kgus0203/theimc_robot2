cmake_minimum_required(VERSION 3.8)
project(theimc_bt_nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(interfaces_pkg REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/clear_resume_target.cpp
  src/drive_cmd_vel.cpp
  src/drive_until_rail_state.cpp
  src/for_each_rail.cpp
  src/get_location_pose.cpp
  src/get_rail_pose.cpp
  src/go_to_pose.cpp
  src/is_battery_low.cpp
  src/load_resume_target.cpp
  src/publish_rail_command.cpp
  src/rail_approach.cpp
  src/rail_enter.cpp
  src/rail_exit.cpp
  src/rail_move_to_distance.cpp
  src/register_nodes.cpp
  src/return_home_requested.cpp
  src/save_rail_progress.cpp
  src/toggle_docking.cpp
  src/wait_for_charge.cpp
  src/wait_for_mission_trigger.cpp
)

target_link_libraries(${PROJECT_NAME}
  yaml-cpp
)

target_compile_definitions(${PROJECT_NAME}
  PRIVATE BT_PLUGIN_EXPORT
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  behaviortree_cpp_v3
  geometry_msgs
  interfaces_pkg
  nav2_behavior_tree
  nav2_msgs
  nav_msgs
  rclcpp
  rclcpp_action
  sensor_msgs
  std_msgs
  std_srvs
  yaml_cpp_vendor
)

add_executable(
  mission_bt_runner
  src/mission_bt_runner.cpp
)

ament_target_dependencies(mission_bt_runner
  ament_index_cpp
  behaviortree_cpp_v3
  geometry_msgs
  interfaces_pkg
  nav2_behavior_tree
  rclcpp
  std_msgs
)

pluginlib_export_plugin_description_file(
  nav2_behavior_tree
  plugins.xml
)

install(
  TARGETS
    ${PROJECT_NAME}
    mission_bt_runner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch bt_xml config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_export_dependencies(
  ament_index_cpp
  behaviortree_cpp_v3
  geometry_msgs
  interfaces_pkg
  nav2_behavior_tree
  nav2_msgs
  nav_msgs
  rclcpp
  rclcpp_action
  sensor_msgs
  std_msgs
  std_srvs
  yaml_cpp_vendor
)

ament_package()
