cmake_minimum_required(VERSION 3.8)
project(vikit_common)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Set build flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -D_LINUX -D_REENTRANT -march=native -Wno-unused-variable -Wno-unused-but-set-variable -Wno-unknown-pragmas")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(fmt REQUIRED)

# Find and configure Eigen3
find_package(Eigen3 REQUIRED)
if(NOT EIGEN3_FOUND)
  message(FATAL_ERROR "Eigen3 not found.")
endif()

# Find Sophus
find_package(Sophus REQUIRED)
if(NOT Sophus_FOUND)
    # Try pkg-config as fallback
    find_package(PkgConfig)
    pkg_check_modules(Sophus REQUIRED sophus)
endif()

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
  ${fmt_INCLUDE_DIRS}
)

# Add definitions for Eigen
add_definitions(-DEIGEN_NO_DEBUG)

# Set source files
set(SOURCEFILES
  src/atan_camera.cpp
  src/omni_camera.cpp
  src/math_utils.cpp
  src/vision.cpp
  src/performance_monitor.cpp
  src/robust_cost.cpp
  src/user_input_thread.cpp
  src/pinhole_camera.cpp
  src/equidistant_camera.cpp
  src/polynomial_camera.cpp
  src/homography.cpp
  src/img_align.cpp
)

# Create vikit library
add_library(${PROJECT_NAME} SHARED ${SOURCEFILES})

# Add dependencies
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  fmt
)

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
  ${Sophus_LIBRARIES}
  fmt::fmt
)

# Add Sophus include directories to target
target_include_directories(${PROJECT_NAME} PUBLIC
  ${Sophus_INCLUDE_DIRS}
)

# Tests
add_executable(test_vk_common_camera test/test_camera.cpp)
target_link_libraries(test_vk_common_camera ${PROJECT_NAME} fmt::fmt)

add_executable(test_vk_common_triangulation test/test_triangulation.cpp)
target_link_libraries(test_vk_common_triangulation ${PROJECT_NAME} fmt::fmt)

add_executable(test_vk_common_patch_score test/test_patch_score.cpp)
target_link_libraries(test_vk_common_patch_score ${PROJECT_NAME} fmt::fmt)

# Install
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Export dependencies
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  Eigen3
  OpenCV
  Sophus
  fmt
)

ament_package()