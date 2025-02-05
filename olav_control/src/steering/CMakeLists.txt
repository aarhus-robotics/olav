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

# STEERING CONTROLLER NODE
# ------------------------
message(STATUS "Building the steering controller node ...")

# > Find external packages
# ------------------------
find_package(diagnostic_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(olav_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# > Add targets
# -------------
add_executable(${PROJECT_NAME}_steering_controller_node node.cpp executor.cpp)

# > Include directories
# ---------------------
target_include_directories(
  ${PROJECT_NAME}_steering_controller_node
  PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)

# > Link external dependencies
# ----------------------------
ament_target_dependencies(
  ${PROJECT_NAME}_steering_controller_node
  diagnostic_msgs
  Eigen3
  nav_msgs
  olav_interfaces
  rclcpp
  std_msgs
  std_srvs)
target_link_libraries(${PROJECT_NAME}_steering_controller_node
                      ${PROJECT_NAME}_core)

# > Define install rules
# ----------------------
install(TARGETS ${PROJECT_NAME}_steering_controller_node
        DESTINATION lib/${PROJECT_NAME})
