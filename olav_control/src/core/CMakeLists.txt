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

# CORE CONTROL LIBRARY
# --------------------
message(STATUS "Building the core control library ...")

# > Find external packages
# ------------------------
find_package(Eigen3 REQUIRED)

# > Add targets
# -------------
add_library(${PROJECT_NAME}_core controllers/pid.cpp controllers/preset.cpp
                                 filters/savitzky_golay.cpp)

# > Include directories
# ---------------------
target_include_directories(
  ${PROJECT_NAME}_core PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                              $<INSTALL_INTERFACE:include>)

# > Link external dependencies
# ----------------------------
ament_target_dependencies(${PROJECT_NAME}_core Eigen3)

# > Define install rules
# ----------------------
install(TARGETS ${PROJECT_NAME}_core DESTINATION lib/${PROJECT_NAME})
