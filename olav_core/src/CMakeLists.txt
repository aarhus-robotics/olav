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

# CORE LIBRARY
# ------------
message(STATUS "Building the core library ...")

# > Find external packages
# ------------------------
find_package(Eigen3 REQUIRED)

# > Add targets
# -------------
add_library(
  ${PROJECT_NAME} filters/low_pass_filter.cpp filters/one_euro_filter.cpp
                  filters/outlier_resistant_filter.cpp math/cubic_spline.cpp)

# > Include directories
# ---------------------
target_include_directories(
  ${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                         $<INSTALL_INTERFACE:include>)

# > Link external dependencies
# ----------------------------
ament_target_dependencies(${PROJECT_NAME} Eigen3)

# > Define install rules
# ----------------------
install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})
