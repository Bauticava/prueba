cmake_minimum_required(VERSION 3.8)
project(entity_spawner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/LoadEntity.srv"
  "srv/SpawnEntity.srv"
  "srv/RemoveEntity.srv"
)

add_executable(entity_spawner_node src/entity_spawner_node.cpp)
ament_target_dependencies(entity_spawner_node
  geometric_shapes
  geometry_msgs
  moveit_ros_planning_interface
  rclcpp
  shape_msgs
  tf2_geometry_msgs
  ros_gz_interfaces
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(entity_spawner_node "${cpp_typesupport_target}")

install(TARGETS entity_spawner_node
  DESTINATION lib/${PROJECT_NAME})

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

ament_export_dependencies(
  geometric_shapes
  geometry_msgs
  moveit_ros_planning_interface
  rclcpp
  ros_gz_interfaces
  rosidl_default_runtime
  shape_msgs
  tf2_geometry_msgs
)

ament_package()
