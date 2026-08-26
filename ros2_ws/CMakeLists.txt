cmake_minimum_required(VERSION 3.5)
project(tai_robot_one)

# Export the exact compiler flags and versioned ROS / Gazebo include paths for
# VS Code IntelliSense and other C++ tooling.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# The ros2_control Jazzy hardware API and this serial backend use C++17.
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz_transport_vendor REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz_msgs_vendor REQUIRED)
find_package(gz-msgs10 REQUIRED)

# Fast DDS is the validated middleware for the current WSL mirrored-network
# setup and the Raspberry Pi deployment. Install a workspace environment hook
# so every terminal that sources install/setup.bash selects the same RMW.
# Preserve an explicit user override for deployments that require another RMW.
ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/env/tai_robot_one_middleware.sh.in"
)

add_library(tai_robot_one_hardware SHARED
  src/tai_robot_serial_system.cpp
)
target_compile_features(tai_robot_one_hardware PUBLIC cxx_std_17)
target_include_directories(tai_robot_one_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(tai_robot_one_hardware PUBLIC
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(
  hardware_interface tai_robot_one_hardware.xml)

# Rendering-independent 2D lidar for Gazebo Harmonic.  It uses DART's
# RaycastData component, allowing the RGB-D camera to stay on Ogre1 under WSL
# without triggering Ogre1's broken GPU-ray implementation.
add_library(tai_physics_lidar SHARED
  src/tai_physics_lidar.cpp
)
target_compile_features(tai_physics_lidar PUBLIC cxx_std_17)
target_link_libraries(tai_physics_lidar PUBLIC
  gz-sim8::gz-sim8
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY config description docs launch udev web worlds
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS tai_robot_one_hardware tai_physics_lidar
  EXPORT export_tai_robot_one_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  PROGRAMS
    scripts/cmd_vel_stamper
    scripts/astra_sim_range_filter
    scripts/keyboard_teleop
    scripts/lift_position_controller
    scripts/phone_teleop_server
    scripts/imu_visualizer
    scripts/test_robot_motion
  DESTINATION lib/${PROJECT_NAME}
)

# The GitHub esp32_imu_bridge package is retained under src/ for provenance,
# but this repository is itself one ament package, so colcon does not descend
# into that nested package. Install its node as part of tai_robot_one so the
# bridge is actually runnable from this workspace.
install(
  PROGRAMS
    src/esp32_imu_bridge/esp32_imu_bridge/esp32_bridge.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME esp32_imu_bridge
)

ament_export_targets(export_tai_robot_one_hardware HAS_LIBRARY_TARGET)
ament_export_dependencies(
  hardware_interface pluginlib rclcpp rclcpp_lifecycle
  gz_sim_vendor gz_transport_vendor gz_msgs_vendor)

ament_package()
