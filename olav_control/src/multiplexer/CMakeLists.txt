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

# CONTROL MULTIPLEXER NODE
# ------------------------
message(STATUS "Building the control multiplexer node ...")

# > Find external packages
# ------------------------
find_package(ackermann_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(olav_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# > Add targets
# -------------
add_executable(${PROJECT_NAME}_control_multiplexer_node control_mode.cpp
                                                        executor.cpp node.cpp)

# > Include directories
# ---------------------
target_include_directories(
  ${PROJECT_NAME}_control_multiplexer_node
  PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)

# > Link external dependencies
# ----------------------------
ament_target_dependencies(
  ${PROJECT_NAME}_control_multiplexer_node
  ackermann_msgs
  diagnostic_msgs
  olav_interfaces
  rclcpp
  std_msgs
  std_srvs)

# > Define install rules
# ----------------------
install(TARGETS ${PROJECT_NAME}_control_multiplexer_node
        DESTINATION lib/${PROJECT_NAME})
