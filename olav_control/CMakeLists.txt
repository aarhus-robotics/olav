# cmake-format: off
 #############################################################################
 #                            _     _     _     _                            #
 #                           / \   / \   / \   / \                           #
 #                          ( O ) ( L ) ( A ) ( V )                          #
 #                           \_/   \_/   \_/   \_/                           #
 #                                                                           #
 #                  OLAV: Off-Road Light Autonomous Vehicle                  #
 #############################################################################
# cmake-format: on

# CMake options
# -------------
cmake_minimum_required(VERSION 3.8)

project(olav_control)

# Compiler options
# ----------------
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES Clang)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Package dependencies
# --------------------
find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(rclpy REQUIRED)

# Build subdirectories
# --------------------
add_subdirectory(src/core)
add_subdirectory(src/drive_by_wire)
add_subdirectory(src/multiplexer)
add_subdirectory(src/speed)
add_subdirectory(src/steering)

# Install targets
# ---------------
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

# Ament package options
# ---------------------
ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(Eigen3)
ament_package()
