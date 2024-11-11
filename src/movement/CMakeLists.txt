cmake_minimum_required(VERSION 3.8)
project(movement)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
moveit_package()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(trajectory_msgs REQUIRED)

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
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

add_executable(arm_movement src/arm_movement.cpp)
ament_target_dependencies(arm_movement rclcpp std_msgs moveit moveit_msgs moveit_ros_planning_interface graph_msgs moveit_visual_tools interfaces)

add_executable(arm_brain src/arm_brain.cpp)
ament_target_dependencies(arm_brain rclcpp std_msgs interfaces)

add_executable(hybrid_planning_demo_node src/hybrid_planning_demo_node.cpp)
ament_target_dependencies(hybrid_planning_demo_node 
  rclcpp 
  std_msgs 
  moveit_core 
  moveit_msgs 
  moveit_ros_planning 
  moveit_ros_planning_interface 
  moveit_visual_tools 
  graph_msgs 
  tf2_ros 
  interfaces)
target_link_libraries(hybrid_planning_demo_node ${LIBRARIES})

install(TARGETS
arm_movement
arm_brain
hybrid_planning_demo_node
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
