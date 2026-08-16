cmake_minimum_required(VERSION 3.8)
project(fast_calib)

set(CMAKE_AUTOMOC ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

find_package(PCL 1.10 REQUIRED)
find_package(OpenCV REQUIRED)

# Include directories
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
# Set build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if (CMAKE_BUILD_TYPE STREQUAL "Release")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
endif()

# Create executable
add_executable(fast_calib src/main.cpp)
add_executable(grab_hikvision_png tools/grab_hikvision_png.cpp)
add_executable(hikvision_image_publisher tools/hikvision_image_publisher.cpp)
add_executable(set_livox_ip tools/set_livox_ip.cpp)
add_executable(manual_lidar_centers_calib tools/manual_lidar_centers_calib.cpp)
add_executable(prepare_static_cloud tools/prepare_static_cloud.cpp)
add_library(fast_calib_rviz_plugins SHARED
  src/board_polygon_selection_tool.cpp
  include/fast_calib/board_polygon_selection_tool.hpp
)

# Link libraries
ament_target_dependencies(fast_calib
  rclcpp
  sensor_msgs
  pcl_conversions
  rosbag2_cpp
)

target_link_libraries(fast_calib
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

ament_target_dependencies(manual_lidar_centers_calib
  rclcpp
  sensor_msgs
  pcl_conversions
  rosbag2_cpp
)

target_link_libraries(manual_lidar_centers_calib
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

ament_target_dependencies(prepare_static_cloud
  rclcpp
  sensor_msgs
  pcl_conversions
  rosbag2_cpp
)

target_link_libraries(prepare_static_cloud
  ${PCL_LIBRARIES}
)

target_include_directories(fast_calib_rviz_plugins PRIVATE include)
ament_target_dependencies(fast_calib_rviz_plugins
  rclcpp
  sensor_msgs
  pcl_conversions
  pluginlib
  rviz_common
  rviz_rendering
)
target_link_libraries(fast_calib_rviz_plugins
  ${PCL_LIBRARIES}
  Qt5::Widgets
)
pluginlib_export_plugin_description_file(rviz_common rviz_plugin/fast_calib_plugins.xml)

target_include_directories(grab_hikvision_png PRIVATE
  /opt/MVS/include
  ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(hikvision_image_publisher
  rclcpp
  sensor_msgs
)

target_include_directories(hikvision_image_publisher PRIVATE
  /opt/MVS/include
  ${OpenCV_INCLUDE_DIRS}
)

target_link_directories(hikvision_image_publisher PRIVATE /opt/MVS/lib/64)

target_link_libraries(hikvision_image_publisher
  ${OpenCV_LIBRARIES}
  MvCameraControl
)

set_target_properties(hikvision_image_publisher PROPERTIES
  INSTALL_RPATH "/opt/MVS/lib/64"
)

target_link_directories(grab_hikvision_png PRIVATE /opt/MVS/lib/64)

target_link_libraries(grab_hikvision_png
  ${OpenCV_LIBRARIES}
  MvCameraControl
)

set_target_properties(grab_hikvision_png PROPERTIES
  INSTALL_RPATH "/opt/MVS/lib/64"
)

target_link_libraries(set_livox_ip
  livox_lidar_sdk_shared
)

# Install
install(TARGETS
  fast_calib
  grab_hikvision_png
  hikvision_image_publisher
  set_livox_ip
  manual_lidar_centers_calib
  prepare_static_cloud
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS fast_calib_rviz_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(PROGRAMS
  scripts/capture_and_run_scene.sh
  scripts/board_polygon_selection_processor.py
  scripts/diagnose_board_plane.py
  scripts/extract_image_from_rosbag.py
  scripts/interactive_calibration_from_bag.sh
  scripts/interactive_calibration_workflow.sh
  scripts/interactive_lidar_hole_editor.py
  scripts/publish_detected_holes.py
  scripts/publish_static_cloud_with_holes.py
  scripts/record_calibration_bag.sh
  scripts/refine_lidar_hole_seeds.py
  scripts/run_fast_calib_scene.sh
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  config
  rviz_cfg
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
