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

project(olav_description)

# Compiler options
# ----------------
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Package dependencies
# --------------------
find_package(ament_cmake REQUIRED)

# Install targets
# ---------------
install(DIRECTORY "urdf" "config" "launch" DESTINATION "share/${PROJECT_NAME}/")

# Ament package options
# ---------------------
ament_package()
