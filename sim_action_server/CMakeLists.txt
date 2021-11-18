cmake_minimum_required(VERSION 3.5)
project(sim_action_server)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_library(sim_action_server SHARED
src/sim_action_server.cpp
)
ament_target_dependencies(sim_action_server
  rclcpp
  rclcpp_action
  control_msgs
)

ament_export_targets(sim_action_server HAS_LIBRARY_TARGET)
target_include_directories(sim_action_server
PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)




install(
  DIRECTORY include/
  DESTINATION include
)
  
install(
  TARGETS sim_action_server
  EXPORT sim_action_server     # <-- documentation prepends export_
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(sim_action_server)

ament_package()
