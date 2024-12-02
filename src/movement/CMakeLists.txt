cmake_minimum_required(VERSION 3.8)
project(movement)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(graph_msgs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(arm_movement src/arm_movement.cpp)
ament_target_dependencies(arm_movement rclcpp std_msgs moveit moveit_msgs moveit_ros_planning_interface graph_msgs moveit_visual_tools interfaces)

add_executable(arm_brain src/arm_brain.cpp)
ament_target_dependencies(arm_brain rclcpp std_msgs interfaces)

install(TARGETS
arm_movement
arm_brain
DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
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
