cmake_minimum_required(VERSION 3.8)
project(sim_environment)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# find Ignition Gazebo (Fortress)
find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-transport11 REQUIRED)
find_package(ignition-common4 REQUIRED)
find_package(ignition-math6 REQUIRED)
find_package(ignition-plugin1 REQUIRED)

# ========== OmniDrive plugin ==========
set(OMNI_DRIVE_HEADERS
  include/${PROJECT_NAME}/OmniDrive.hh
)

set(OMNI_DRIVE_SOURCES
  src/OmniDrive.cc
)

add_library(OmniDrive SHARED
  ${OMNI_DRIVE_HEADERS}
  ${OMNI_DRIVE_SOURCES}
)

target_include_directories(OmniDrive
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
    $<INSTALL_INTERFACE:include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_link_libraries(OmniDrive
  PUBLIC
    ignition-gazebo6::ignition-gazebo6
    ignition-transport11::ignition-transport11
    ignition-common4::ignition-common4
    ignition-math6::ignition-math6
    ignition-plugin1::ignition-plugin1
)

# Ensure plugin loads correctly (C++17 required by Ignition)
target_compile_features(OmniDrive PUBLIC cxx_std_17)

# ========== install ==========
install(
  TARGETS OmniDrive
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY urdf launch rviz meshes world config map
  DESTINATION share/${PROJECT_NAME}/
)

install(
  PROGRAMS scripts/fix_scan_frame.py scripts/map_relay.py scripts/odom_to_tf.py scripts/cmd_vel_visualizer.py
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
