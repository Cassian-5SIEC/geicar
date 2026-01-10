cmake_minimum_required(VERSION 3.8)
project(network_hmi)

# Compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---- Find Standard Dependencies ----
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(PkgConfig REQUIRED)

# ---- OpenCV Configuration Logic ----

# We check if the specific Jetson library exists. 
# If yes, we use manual paths. If no, we use standard find_package.
set(JETSON_OPENCV_CHECK_FILE "/lib/aarch64-linux-gnu/libopencv_core.so.4.5d")

if(EXISTS "${JETSON_OPENCV_CHECK_FILE}")
  message(STATUS "Detected Jetson Orin custom OpenCV libs. Configuring manually...")

  set(OpenCV_INCLUDE_DIRS "/usr/include/opencv4")
  
  set(OpenCV_LIBRARIES
    "/lib/aarch64-linux-gnu/libopencv_core.so.4.5d"
    "/lib/aarch64-linux-gnu/libopencv_imgproc.so.4.5d"
    "/lib/aarch64-linux-gnu/libopencv_imgcodecs.so.4.5d"
    "/lib/aarch64-linux-gnu/libopencv_highgui.so.4.5d"
    # Add others here if needed
  )
  
  # Set RPATH for Jetson
  set(CMAKE_INSTALL_RPATH "${OpenCV_LIB_PATH};/opt/ros/$ENV{ROS_DISTRO}/lib")

else()
  message(STATUS "Jetson custom libs not found. Using standard system OpenCV...")
  
  # Standard PC Configuration
  find_package(OpenCV REQUIRED)
  
  # Standard RPATH configuration
  set(CMAKE_INSTALL_RPATH "/opt/ros/$ENV{ROS_DISTRO}/lib")
endif()

message(STATUS "Final OpenCV Include: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "Final OpenCV Libs: ${OpenCV_LIBRARIES}")

# ---- RPATH Settings (Common) ----
set(CMAKE_SKIP_BUILD_RPATH FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# ---- GStreamer Dependencies ----
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)
pkg_check_modules(GSTREAMER_BASE REQUIRED gstreamer-base-1.0)
pkg_check_modules(GST REQUIRED gstreamer-app-1.0)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${GSTREAMER_INCLUDE_DIRS}
  ${GSTREAMER_BASE_INCLUDE_DIRS}
  ${GST_INCLUDE_DIRS}
)

link_directories(
  ${GSTREAMER_LIBRARY_DIRS}
  ${GSTREAMER_BASE_LIBRARY_DIRS}
  ${GST_LIBRARY_DIRS}
)

# ---- Library with components ----
add_library(network_hmi_lib
  src/client_session.cpp
  src/h264_streamer.cpp
  src/shared_client_info.cpp
  src/shared_vehicle_state.cpp
  src/tcp_control_server.cpp
  src/udp_data_receiver.cpp
  src/udp_data_sender.cpp
  src/tcp_udp_bridge_node.cpp
)

target_include_directories(network_hmi_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(network_hmi_lib
  PUBLIC
    nlohmann_json::nlohmann_json
    ${cv_bridge_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${GSTREAMER_LIBRARIES}
    ${GSTREAMER_BASE_LIBRARIES}
    ${GST_LIBRARIES}
)

ament_target_dependencies(network_hmi_lib
  PUBLIC
  rclcpp
  interfaces
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  sensor_msgs
  cv_bridge
)

# ---- Executable ----
add_executable(bridge_node src/main.cpp)

target_link_libraries(bridge_node
  network_hmi_lib
)

# ---- Install rules ----
install(TARGETS
  bridge_node
  network_hmi_lib
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

ament_package()