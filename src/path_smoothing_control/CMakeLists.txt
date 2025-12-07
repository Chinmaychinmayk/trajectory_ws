cmake_minimum_required(VERSION 3.8)
project(path_smoothing_control LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

# ------------------- Dependencies -------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(ament_cmake_gtest REQUIRED)

include_directories(include)

# ------------------- Libraries -------------------
add_library(path_smoother_lib STATIC src/path_smoother.cpp)
add_library(trajectory_generator_lib STATIC src/trajectory_generator.cpp)
add_library(trajectory_controller_lib STATIC src/trajectory_controller.cpp)

ament_target_dependencies(path_smoother_lib          rclcpp nav_msgs geometry_msgs)
ament_target_dependencies(trajectory_generator_lib   rclcpp nav_msgs geometry_msgs)
ament_target_dependencies(trajectory_controller_lib  rclcpp nav_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs)

# ------------------- Main Node -------------------
add_executable(path_smoothing_node src/path_smoothing_node.cpp)

target_link_libraries(path_smoothing_node
  path_smoother_lib
  trajectory_generator_lib
  trajectory_controller_lib
)

# THIS WAS MISSING → fixes the undefined references
ament_target_dependencies(path_smoothing_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# ------------------- Tests -------------------
if(BUILD_TESTING)
  ament_add_gtest(test_controller test/test_controller.cpp)

  target_link_libraries(test_controller
    path_smoother_lib
    trajectory_generator_lib
    trajectory_controller_lib
  )

  ament_target_dependencies(test_controller
    rclcpp
    std_msgs
    geometry_msgs
    nav_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
  )
endif()

# ------------------- Install -------------------
install(TARGETS path_smoothing_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME}/)
install(PROGRAMS 
  scripts/visualize_trajectory.py 
  scripts/mock_sim_node.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()

# Install URDF files
