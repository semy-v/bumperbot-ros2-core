cmake_minimum_required(VERSION 3.8)
project(bumperbot_motion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# ament_python_install_package(${PROJECT_NAME})

include_directories(include)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  nav2_core
  nav2_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pluginlib
)

set(plugin_dependencies
  ${dependencies}
  nav2_util
)

add_library(motion_control_plugins SHARED
  src/core/pd_pure_pursuit_core.cpp
  src/plugin/pd_pure_pursuit_plugin.cpp
)
ament_target_dependencies(motion_control_plugins ${plugin_dependencies})

pluginlib_export_plugin_description_file(nav2_core motion_control_plugins.xml)

add_executable(motion_control_server
  src/motion_control_server.cpp
  src/motion_control_app.cpp
)
ament_target_dependencies(motion_control_server ${dependencies})

install(TARGETS
  motion_control_server
  motion_control_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(
  motion_control_plugins
)
ament_export_dependencies(${plugin_dependencies})
ament_package()