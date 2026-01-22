cmake_minimum_required(VERSION 3.8)
project(ecosense_arm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(behaviortree_ros2 REQUIRED)
find_package(behaviortree_cpp REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Add test executable
add_executable(demo_pick_place  test/demo_pick_place.cpp)
add_executable(demo_bt_arm  test/demo_bt_arm.cpp)

# This is compiled only to check if there are errors in the header file
# library will not be exported
add_library(${PROJECT_NAME} SHARED
  src/bt_arm_nodes.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_action
  behaviortree_cpp
  behaviortree_ros2
  moveit_ros_planning_interface
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

ament_target_dependencies(demo_pick_place 
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  behaviortree_cpp
  behaviortree_ros2
)

target_link_libraries(demo_bt_arm ${PROJECT_NAME})

ament_target_dependencies(demo_bt_arm 
  rclcpp
  behaviortree_cpp
  behaviortree_ros2
)

# Install the library
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ 
  DESTINATION include
  )

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

install(DIRECTORY 
  config
  urdf
  models
  tree
  DESTINATION share/${PROJECT_NAME}
)

install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
install(TARGETS demo_pick_place DESTINATION lib/${PROJECT_NAME})
install(TARGETS demo_bt_arm DESTINATION lib/${PROJECT_NAME})

ament_export_libraries(${PROJECT_NAME})
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  behaviortree_cpp
  behaviortree_ros2
  moveit_ros_planning_interface
)

ament_package()
