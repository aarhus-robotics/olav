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

# POWERTRAIN INTERFACE NODE
# -------------------------
message(STATUS "Building the powertrain interface node ...")

# > Find external packages
# ------------------------
find_package(diagnostic_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(olav_core REQUIRED)
find_package(olav_interfaces REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(serial REQUIRED)

# > Add targets
# -------------
add_executable(${PROJECT_NAME}_powertrain_interface_node
               exceptions.cpp executor.cpp interface.cpp node.cpp)

# > Include directories
# ---------------------
target_include_directories(
  ${PROJECT_NAME}_powertrain_interface_node
  PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)

# > Link external dependencies
# ----------------------------
ament_target_dependencies(
  ${PROJECT_NAME}_powertrain_interface_node
  diagnostic_msgs
  olav_core
  olav_interfaces
  nav_msgs
  rclcpp
  rclcpp_action
  serial)
target_link_libraries(${PROJECT_NAME}_powertrain_interface_node
                      ${PROJECT_NAME}_serial_device_interface_library)

# > Define install rules
# ----------------------
install(TARGETS ${PROJECT_NAME}_powertrain_interface_node
        DESTINATION lib/${PROJECT_NAME})
