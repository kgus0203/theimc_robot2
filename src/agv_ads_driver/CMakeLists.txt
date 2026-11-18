cmake_minimum_required(VERSION 3.10)
project(agv_ads_driver)

# ============================================================
# Compiler
# ============================================================
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ============================================================
# ROS2 dependencies
# ============================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Threads REQUIRED)

# ============================================================
# Beckhoff / AdsLib paths
#
# Default layout:
#   /home/jeff/AGVSource/
#     ├── ADS/
#     └── Beckhoff/
#
# If the location changes:
#   colcon build --cmake-args \
#     -DAGV_SOURCE_ROOT=/another/path/AGVSource
# ============================================================
set(
  AGV_SOURCE_ROOT
  "/home/jeff/AGVSource"
  CACHE PATH
  "Root directory of AGVSource"
)

set(BECKHOFF_DIR "${AGV_SOURCE_ROOT}/Beckhoff")
set(ADS_DIR      "${AGV_SOURCE_ROOT}/ADS")

set(BECKHOFF_INCLUDE_DIR "${BECKHOFF_DIR}/include")
set(BECKHOFF_SRC_DIR     "${BECKHOFF_DIR}/src")
set(ADSLIB_INCLUDE_DIR   "${ADS_DIR}/AdsLib")
set(ADSLIB_STATIC_LIB    "${BECKHOFF_DIR}/lib/libAdsLib.a")

# ============================================================
# Sanity checks
# ============================================================
if(NOT EXISTS "${BECKHOFF_INCLUDE_DIR}/TcMCApi.h")
  message(FATAL_ERROR
    "TcMCApi.h not found: ${BECKHOFF_INCLUDE_DIR}/TcMCApi.h")
endif()

if(NOT EXISTS "${BECKHOFF_SRC_DIR}/TcMC.cpp")
  message(FATAL_ERROR
    "TcMC.cpp not found: ${BECKHOFF_SRC_DIR}/TcMC.cpp")
endif()

if(NOT EXISTS "${BECKHOFF_SRC_DIR}/TcMCApi.cpp")
  message(FATAL_ERROR
    "TcMCApi.cpp not found: ${BECKHOFF_SRC_DIR}/TcMCApi.cpp")
endif()

if(NOT EXISTS "${ADSLIB_STATIC_LIB}")
  message(FATAL_ERROR
    "AdsLib static library not found: ${ADSLIB_STATIC_LIB}\n"
    "Build AdsLib first and confirm libAdsLib.a exists.")
endif()

if(NOT EXISTS "${BECKHOFF_SRC_DIR}/TcSystem.cpp")
  message(FATAL_ERROR
    "TcSystem.cpp not found: ${BECKHOFF_SRC_DIR}/TcSystem.cpp")
endif()

if(NOT EXISTS "${BECKHOFF_INCLUDE_DIR}/TcSystem.h")
  message(FATAL_ERROR
    "TcSystem.h not found: ${BECKHOFF_INCLUDE_DIR}/TcSystem.h")
endif()

# ============================================================
# Driver source
#
# Existing project used executable name 'agv_node'.
# Keep that name so current launch files do not need to change.
#
# Preferred source filename:
#   src/agv_ads_driver.cpp
#
# For compatibility, src/agv_node.cpp is also accepted.
# ============================================================
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/agv_ads_driver.cpp")
  set(AGV_DRIVER_SOURCE
    "${CMAKE_CURRENT_SOURCE_DIR}/src/agv_ads_driver.cpp")
elseif(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/agv_node.cpp")
  set(AGV_DRIVER_SOURCE
    "${CMAKE_CURRENT_SOURCE_DIR}/src/agv_node.cpp")
else()
  message(FATAL_ERROR
    "AGV driver source not found.\n"
    "Place the driver at either:\n"
    "  src/agv_ads_driver.cpp\n"
    "or\n"
    "  src/agv_node.cpp")
endif()

# ============================================================
# Executable
# ============================================================
add_executable(
  agv_node
  ${AGV_DRIVER_SOURCE}
  "${BECKHOFF_SRC_DIR}/TcMC.cpp"
  "${BECKHOFF_SRC_DIR}/TcMCApi.cpp"
  "${BECKHOFF_SRC_DIR}/TcSystem.cpp"
)

target_include_directories(
  agv_node
  PRIVATE
    "${BECKHOFF_INCLUDE_DIR}"
    "${ADSLIB_INCLUDE_DIR}"
)

ament_target_dependencies(
  agv_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_srvs
)

target_link_libraries(
  agv_node
  "${ADSLIB_STATIC_LIB}"
  Threads::Threads
)

# ============================================================
# Install
# ============================================================
install(
  TARGETS agv_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch/config directories only if they exist.
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
  )
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config")
  install(
    DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
  )
endif()

ament_package()
