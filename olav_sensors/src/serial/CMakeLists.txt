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

# SERIAL DEVICE INTERFACE LIBRARY
# -------------------------------
message(STATUS "Building the serial device interface library ...")

# > Find external packages
# ------------------------
find_package(nav_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(olav_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(serial REQUIRED)

# > Add targets
# -------------
add_library(${PROJECT_NAME}_serial_device_interface_library
            interface.cpp exceptions.cpp status_message.cpp)

# > Include directories
# ---------------------
target_include_directories(
  ${PROJECT_NAME}_serial_device_interface_library
  PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)

# > Link external dependencies
# ----------------------------
ament_target_dependencies(${PROJECT_NAME}_serial_device_interface_library
                          olav_interfaces nav_msgs rclcpp serial)
target_link_libraries(${PROJECT_NAME}_serial_device_interface_library
                      nlohmann_json::nlohmann_json)

# > Define install rules
# ----------------------
install(TARGETS ${PROJECT_NAME}_serial_device_interface_library
        DESTINATION lib/${PROJECT_NAME})
