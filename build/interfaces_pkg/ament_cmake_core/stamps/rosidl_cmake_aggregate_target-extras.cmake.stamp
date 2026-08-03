# generated from rosidl_cmake/cmake/rosidl_cmake_aggregate_target-extras.cmake.in

# Create a convenience aggregate target interfaces_pkg::interfaces_pkg
# that links all generated interface targets, so downstream packages can use
# a single modern CMake target name instead of ${interfaces_pkg_TARGETS}.
if(interfaces_pkg_TARGETS AND NOT TARGET interfaces_pkg::interfaces_pkg)
  add_library(interfaces_pkg::interfaces_pkg INTERFACE IMPORTED)
  set_target_properties(interfaces_pkg::interfaces_pkg PROPERTIES
    INTERFACE_LINK_LIBRARIES "${interfaces_pkg_TARGETS}")
endif()
