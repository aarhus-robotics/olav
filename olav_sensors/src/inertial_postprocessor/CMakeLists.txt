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

# INERTIAL POSTPROCESSOR NODE
# ---------------------------
message(STATUS "Building the inertial postprocessor node ...")

# > Find external packages
# ------------------------
find_package(nav_msgs REQUIRED)
find_package(olav_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)

# > Add targets
# -------------
add_executable(${PROJECT_NAME}_inertial_postprocessor_node executor.cpp
                                                           node.cpp)

# > Include directories
# ---------------------
target_include_directories(
  ${PROJECT_NAME}_inertial_postprocessor_node
  PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)

# > Link external dependencies
# ----------------------------
ament_target_dependencies(${PROJECT_NAME}_inertial_postprocessor_node nav_msgs
                          olav_core rclcpp tf2_ros)

# > Define install rules
# ----------------------
install(TARGETS ${PROJECT_NAME}_inertial_postprocessor_node
        DESTINATION lib/${PROJECT_NAME})
