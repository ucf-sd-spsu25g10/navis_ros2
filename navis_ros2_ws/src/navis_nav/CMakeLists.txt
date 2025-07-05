cmake_minimum_required(VERSION 3.8)
project(navis_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(navis_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(ros2_control REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(gpio_controllers REQUIRED)
find_package(robot_state_publisher REQUIRED)

# Include paths
include_directories(include)

# Executables
add_executable(control_out_node
  src/control_out_node.cpp
)

add_executable(control_action_calc
  src/control_action_calc.cpp
)

add_executable(bag_recorder
  src/bag_recorder.cpp
)

add_executable(waypoint_manager
  src/waypoint_manager.cpp
  src/grocery_locations.cpp
  src/waypoint_orderer.cpp
)

# Include dirs per target
foreach(target IN ITEMS control_out_node waypoint_manager control_action_calc)
  target_include_directories(${target} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  )
  target_compile_features(${target} PUBLIC c_std_99 cxx_std_17)
endforeach()

# Dependencies per target
ament_target_dependencies(waypoint_manager
  rclcpp
  std_msgs
  geometry_msgs
  navis_msgs
  ros2_control
  hardware_interface
  controller_manager
  gpio_controllers
)

ament_target_dependencies(control_out_node 
  rclcpp
  rclcpp_components
  navis_msgs
)

ament_target_dependencies(control_action_calc 
  rclcpp
  rclcpp_components
  nav_msgs
  navis_msgs
)

ament_target_dependencies(bag_recorder
  rclcpp
  sensor_msgs
  rosbag2_cpp
)

# Install executables
install(TARGETS 
  control_out_node
  control_action_calc
  waypoint_manager
  bag_recorder
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch + headers
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  include/
  DESTINATION include/
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

add_executable(haptic_control_node src/haptic_control_node.cpp)
ament_target_dependencies(haptic_control_node rclcpp geometry_msgs navis_msgs nav_msgs tf2)

install(TARGETS
  haptic_control_node
  DESTINATION lib/${PROJECT_NAME}
)
