cmake_minimum_required(VERSION 3.8)
project(bumperbot_firmware VERSION 0.1)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# 1. Try finding libserial via PkgConfig first (usually works on Host/Ubuntu)
find_package(PkgConfig QUIET)
if(PKG_CONFIG_FOUND)
    pkg_check_modules(SERIAL QUIET IMPORTED_TARGET libserial)
endif()

# 2. If PkgConfig failed (like in Yocto), search manually for headers and library
if(NOT SERIAL_FOUND)
    message(STATUS "libserial metadata not found, searching manually...")
    find_path(SERIAL_INCLUDE_DIRS NAMES libserial/SerialPort.h)
    find_library(SERIAL_LIBRARIES NAMES serial)

    if(SERIAL_INCLUDE_DIRS AND SERIAL_LIBRARIES)
        set(SERIAL_FOUND TRUE)
        # Create the same Imported Target that PkgConfig would have created
        if(NOT TARGET PkgConfig::SERIAL)
            add_library(PkgConfig::SERIAL INTERFACE IMPORTED)
            set_target_properties(PkgConfig::SERIAL PROPERTIES
                INTERFACE_INCLUDE_DIRECTORIES "${SERIAL_INCLUDE_DIRS}"
                INTERFACE_LINK_LIBRARIES "${SERIAL_LIBRARIES}"
            )
        endif()
    endif()
endif()

if(NOT TARGET PkgConfig::SERIAL)
  message(FATAL_ERROR "libserial not found! (Tried PkgConfig and manual search)")
endif()

ament_python_install_package(${PROJECT_NAME})

add_library(${PROJECT_NAME} SHARED
  src/diff_drive/diff_drive_interface.cpp
)
target_include_directories(${PROJECT_NAME} PRIVATE
  include
  arduino/diff_drive_controller/protocol
  "${SERIAL_INCLUDE_DIRS}"
)
target_link_libraries(${PROJECT_NAME} PkgConfig::SERIAL)
ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)
pluginlib_export_plugin_description_file(hardware_interface diff_drive_interface.xml)

install(PROGRAMS
  ${PROJECT_NAME}/diff_drive/diff_drive_messages.py
  ${PROJECT_NAME}/diff_drive/serial_transceiver_node.py
  ${PROJECT_NAME}/diff_drive/velocity_cmd_test_runner_node.py
  ${PROJECT_NAME}/imu/mpu6050_driver.py
  ${PROJECT_NAME}/imu/mpu6050_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()
