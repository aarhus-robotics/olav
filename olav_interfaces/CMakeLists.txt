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

project(olav_interfaces)

# Compiler options
# ----------------
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES Clang)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Package dependencies
# --------------------
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# Interface definitions
# ---------------------
rosidl_generate_interfaces(
  ${PROJECT_NAME}
  # Actions
  "action/ShiftGear.action"
  "action/SinusoidalSteering.action"
  # Messages
  "msg/DriveByWirePLCStatus.msg"
  "msg/OdometerMCUStatus.msg"
  "msg/PIDStatus.msg"
  "msg/SetpointStamped.msg"
  # Services
  "srv/SetControlMode.srv"
  "srv/SetPIDGains.srv"
  DEPENDENCIES
  std_msgs)

# Ament package options
# ---------------------
ament_package()
