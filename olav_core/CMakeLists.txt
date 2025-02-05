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

# CMake options
# -------------
cmake_minimum_required(VERSION 3.8)

project(olav_core)

# Compiler options
# ----------------
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES Clang)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Package dependencies
# --------------------
find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)

# Build subdirectories
# --------------------
add_subdirectory(src)

# Install targets
# ---------------
install(DIRECTORY include/ DESTINATION include)

install(
  TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)

# Ament package options
# ---------------------
ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(Eigen3)
ament_export_libraries(${PROJECT_NAME})
ament_export_include_directories(include)
ament_package()
