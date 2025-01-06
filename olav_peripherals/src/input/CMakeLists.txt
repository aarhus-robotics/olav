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

# Gamepad interface node
# ----------------------
message(STATUS "Building the gamepad interface node ...")

# > Gamepad interface packages
find_package(olav_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# > Gamepad interface node target
add_executable(
  ${PROJECT_NAME}_gamepad_interface_node "gamepad_interface_node.cpp"
                                         "gamepad_interface_node_executor.cpp")

# > Gamepad interface node headers
target_include_directories(
  ${PROJECT_NAME}_gamepad_interface_node
  PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)

# > Gamepad interface node ROS dependencies
ament_target_dependencies(${PROJECT_NAME}_gamepad_interface_node
                          olav_interfaces rclcpp sensor_msgs std_srvs)

# > Gamepad interface node targets install
install(TARGETS ${PROJECT_NAME}_gamepad_interface_node
        DESTINATION "lib/${PROJECT_NAME}")
