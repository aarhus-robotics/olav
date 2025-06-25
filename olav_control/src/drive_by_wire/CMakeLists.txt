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

# DRIVE-BY-WIRE INTERFACE NODE
# ----------------------------
message(STATUS "Building the drive-by-wire interface node ...")

# > Find external packages
# ------------------------
find_package(ackermann_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(Modbus REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(olav_core REQUIRED)
find_package(olav_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# > Add targets
# -------------
add_executable(
  ${PROJECT_NAME}_drive_by_wire_interface_node
  control_mode.cpp
  feedback.cpp
  interface.cpp
  exceptions.cpp
  node.cpp
  executor.cpp
  setpoint.cpp)

# > Include directories
# ---------------------
target_include_directories(
  ${PROJECT_NAME}_drive_by_wire_interface_node
  PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)

# > Link external dependencies
# ----------------------------
ament_target_dependencies(
  ${PROJECT_NAME}_drive_by_wire_interface_node
  ackermann_msgs
  diagnostic_msgs
  nav_msgs
  olav_core
  olav_interfaces
  rclcpp
  sensor_msgs
  std_srvs)
target_link_libraries(${PROJECT_NAME}_drive_by_wire_interface_node
                      ${PROJECT_NAME}_core modbus)

# > Define install rules
# ----------------------
install(TARGETS ${PROJECT_NAME}_drive_by_wire_interface_node
        DESTINATION lib/${PROJECT_NAME})
