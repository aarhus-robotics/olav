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

project(olav_peripherals)

# Compiler options
# ----------------
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES Clang)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Package dependencies
# --------------------
find_package(ament_cmake REQUIRED)

# Build subdirectories
# --------------------
add_subdirectory(src/gamepad)

# Python package
# --------------------
ament_python_install_package(${PROJECT_NAME})

# Install targets
# ---------------
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

# > Install Python executables
install(PROGRAMS scripts/control_panel_node.py DESTINATION lib/${PROJECT_NAME})

# Ament package options
# ---------------------
ament_package()
