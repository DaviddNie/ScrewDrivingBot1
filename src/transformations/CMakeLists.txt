cmake_minimum_required(VERSION 3.8)
project(transformations)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(interfaces REQUIRED)
find_package(visualization_msgs REQUIRED)

# add_executable(camera_dy_trans src/camera_dy_trans.cpp)

# ament_target_dependencies(
#   camera_dy_trans rclcpp tf2 tf2_ros geometry_msgs interfaces tf2_ros)

add_executable(ooi_server src/ooi_server.cpp)

ament_target_dependencies(
  ooi_server rclcpp tf2 tf2_ros geometry_msgs interfaces tf2_ros)

install(TARGETS
# camera_dy_trans
  ooi_server
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
