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

# Serial device interface library
# -------------------------------
message(STATUS "Building the serial device interface library ...")

# > Serial device interface library packages
find_package(nav_msgs REQUIRED)
# --- #
find_package(olav_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(serial REQUIRED)
# --- #
find_package(nlohmann_json REQUIRED)

# > Serial device interface library target
add_library(
  ${PROJECT_NAME}_serial_device_interface_library
  "serial_device_interface.cpp" "serial_device_interface_exceptions.cpp"
  "serial_device_status_message.cpp")

# > Serial device interface library headers
target_include_directories(
  ${PROJECT_NAME}_serial_device_interface_library
  PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)

# > Serial device interface library ROS dependencies
ament_target_dependencies(${PROJECT_NAME}_serial_device_interface_library
                          olav_interfaces nav_msgs rclcpp serial)

# > Serial device interface library linked libraries
target_link_libraries(${PROJECT_NAME}_serial_device_interface_library
                      nlohmann_json::nlohmann_json)

# > Serial device interface library targets install
install(TARGETS ${PROJECT_NAME}_serial_device_interface_library
        DESTINATION "lib/${PROJECT_NAME}")
