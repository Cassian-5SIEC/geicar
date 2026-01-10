cmake_minimum_required(VERSION 3.8)
project(car_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs  REQUIRED)

add_executable(car_control_node 
  src/car_control_node.cpp
  src/steeringCmd.cpp
  src/propulsionCmd.cpp
)

add_executable(vehicle_controller 
  src/vehicle_controller.cpp
)

ament_target_dependencies(vehicle_controller rclcpp std_msgs geometry_msgs)
ament_target_dependencies(car_control_node  rclcpp interfaces sensor_msgs std_srvs)
ament_export_include_directories(include)

target_include_directories(car_control_node  PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)

target_include_directories(vehicle_controller PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)


target_compile_features(car_control_node  PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17


install(TARGETS car_control_node 
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  vehicle_controller
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
