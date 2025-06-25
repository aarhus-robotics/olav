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

# HEART NODE
# ----------
message(STATUS "Building the heart node ...")

# > Find external packages
# ------------------------
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)

# > Add targets
# -------------
add_executable(
  ${PROJECT_NAME}_heart_node
  executor.cpp
  node.cpp
)

# > Include directories
# ---------------------
target_include_directories(
  ${PROJECT_NAME}_heart_node
  PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)

# > Link external dependencies
# ----------------------------
ament_target_dependencies(
  ${PROJECT_NAME}_heart_node
  rclcpp
  std_msgs)

# > Define install rules
# ----------------------
install(TARGETS ${PROJECT_NAME}_heart_node
        DESTINATION lib/${PROJECT_NAME})
