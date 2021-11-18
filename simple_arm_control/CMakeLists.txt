cmake_minimum_required(VERSION 3.5.0)
project(simple_arm_control)

# Default to C11
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(NOT WIN32)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_TYPE Debug)

find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(sim_action_server REQUIRED)
find_package(gazebo_msgs REQUIRED)
install(DIRECTORY
  DESTINATION share/${PROJECT_NAME}/
)
include_directories(include)

add_executable(throw_ign src/throw_ign.cpp include/moveit.hpp)
ament_target_dependencies(throw_ign
  rclcpp
  rclcpp_action
  control_msgs
  moveit_ros_planning_interface
  sim_action_server
  gazebo_msgs
)
add_executable(moveit_ign src/moveit_ign.cpp include/moveit.hpp)
ament_target_dependencies(moveit_ign
  rclcpp
  rclcpp_action
  control_msgs
  moveit_ros_planning_interface
  sim_action_server
  gazebo_msgs
)
add_executable(moveit_collision src/moveit_collision.cpp include/moveit.hpp)
ament_target_dependencies(moveit_collision
  rclcpp
  rclcpp_action
  control_msgs
  moveit_ros_planning_interface
  sim_action_server
  gazebo_msgs
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  ament_lint_auto_find_test_dependencies()
endif()

## Install
install(TARGETS throw_ign moveit_collision moveit_ign
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include
  DESTINATION include/${PROJECT_NAME}/)
  

ament_package()
