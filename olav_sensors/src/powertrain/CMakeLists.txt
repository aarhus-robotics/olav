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

# Powertrain interface node
# -------------------------
message(STATUS "Building the powertrain interface node ...")

# > Powertrain interface node packages --- #
find_package(nav_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
# --- #
find_package(olav_interfaces REQUIRED)
find_package(serial REQUIRED)
# --- #
find_package(nlohmann_json REQUIRED)

# > Powertrain interface node target
add_executable(
  ${PROJECT_NAME}_powertrain_interface_node
  "powertrain_interface_node_executor.cpp" "powertrain_interface_node.cpp"
  "powertrain_interface_node_executor.cpp" "powertrain_interface.cpp"
  "powertrain_interface_exceptions.cpp")

# > Powertrain interface node headers
target_include_directories(
  ${PROJECT_NAME}_powertrain_interface_node
  PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)

# > Powertrain interface node ROS dependencies
ament_target_dependencies(${PROJECT_NAME}_powertrain_interface_node
                          olav_interfaces nav_msgs rclcpp rclcpp_action serial)

# > Powertrain interface node linked libraries
target_link_libraries(${PROJECT_NAME}_powertrain_interface_node
                      ${PROJECT_NAME}_serial_device_interface_library)

# > Powertrain interface node targets install
install(TARGETS ${PROJECT_NAME}_powertrain_interface_node
        DESTINATION "lib/${PROJECT_NAME}")
