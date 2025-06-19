cmake_minimum_required(VERSION 3.8)
project(gazebo_differential_drive_robot_4wheel)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gz-transport12 REQUIRED)
find_package(gz-msgs9 REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(gt_bridge_node src/gt_bridge_node.cpp)
ament_target_dependencies(gt_bridge_node
  rclcpp
  geometry_msgs
)

add_executable(reset_and_respawn_node src/reset_and_respawn_node.cpp)
ament_target_dependencies(reset_and_respawn_node rclcpp std_srvs ros_gz_interfaces)

target_link_libraries(gt_bridge_node
  gz-transport12::gz-transport12
  gz-msgs9::gz-msgs9
)

target_link_libraries(reset_and_respawn_node
  gz-transport12::gz-transport12
  gz-msgs9::gz-msgs9
)

install(
  TARGETS gt_bridge_node DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS reset_and_respawn_node DESTINATION lib/${PROJECT_NAME}
)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

#ament_python_install_package(${PROJECT_NAME})

install(
 DIRECTORY launch model config worlds
 DESTINATION share/${PROJECT_NAME}
)


ament_package()
