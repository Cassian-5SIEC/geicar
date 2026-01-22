cmake_minimum_required(VERSION 3.8)
project(bt_ecosense)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(ecosense_arm REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(interfaces REQUIRED)

# Add the Executable
add_executable(bt_ecosense_node src/bt_ecosense.cpp)

# 2. Configure Include Directories (THE CRITICAL PART)
# This adds the 'include' folder to the search path.
# This allows:
#   src/bt_ecosense.cpp -> #include "bt_ecosense/bt_ecosense.hpp"
#   include/bt_ecosense/bt_ecosense.hpp -> #include "action/action1.hpp" (Relative path works automatically)
target_include_directories(bt_ecosense_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 3. Link Dependencies
ament_target_dependencies(bt_ecosense_node
  rclcpp
  behaviortree_cpp
  behaviortree_ros2
  ecosense_arm
  std_srvs
  geometry_msgs
  nav2_msgs
  interfaces
)

install(
  TARGETS bt_ecosense_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    tree
  DESTINATION 
    share/${PROJECT_NAME}
)

ament_package()
