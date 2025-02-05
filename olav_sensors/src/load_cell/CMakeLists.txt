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

# Load cell interface node
# ------------------------
message(STATUS "Building the load cell interface node ...")

# > Load cell interface node packages
find_package(olav_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
# --- #
find_package(nlohmann_json REQUIRED)

# > Load cell interface node target
add_executable(
  ${PROJECT_NAME}_load_cell_interface_node
  "load_cell_interface.cpp" "load_cell_interface_node.cpp"
  "load_cell_interface_node_executor.cpp")

# > Load cell interface node headers
target_include_directories(
  ${PROJECT_NAME}_load_cell_interface_node
  PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)

# > Load cell interface node ROS dependencies
ament_target_dependencies(${PROJECT_NAME}_load_cell_interface_node
                          olav_interfaces rclcpp)

# > Load cell interface node linked libraries
target_link_libraries(
  ${PROJECT_NAME}_load_cell_interface_node
  ${PROJECT_NAME}_serial_device_interface_library nlohmann_json::nlohmann_json)

# > Load cell interface node targets install
install(TARGETS ${PROJECT_NAME}_load_cell_interface_node
        DESTINATION "lib/${PROJECT_NAME}")
