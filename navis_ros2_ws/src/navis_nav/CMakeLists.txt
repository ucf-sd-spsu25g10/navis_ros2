cmake_minimum_required(VERSION 3.8)
project(navis_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(navis_msgs REQUIRED)

add_executable(waypoint_orderer 
  src/waypoint_orderer.cpp
  src/grocery_locations.cpp
)

add_executable(imu_publisher 
  src/imu_publisher.cpp
)

add_executable(control_out_node
  src/control_out_node.cpp
)


target_include_directories(waypoint_orderer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(waypoint_orderer PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS 
  waypoint_orderer 
  imu_publisher
  control_out_node

  DESTINATION lib/${PROJECT_NAME}
)

# Install assets
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_target_dependencies(waypoint_orderer
  navis_msgs
  rclcpp
  rclcpp_components
)

ament_target_dependencies(imu_publisher 
  sensor_msgs
  rclcpp 
)

ament_target_dependencies(control_out_node 
  navis_msgs
  rclcpp
  rclcpp_components 
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
