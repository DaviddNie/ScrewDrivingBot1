cmake_minimum_required(VERSION 3.8)
project(interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/VisionCmd.srv"
  "srv/BrainCmd.srv"
  "srv/EndEffectorCmd.srv"
  "srv/BrainRoutineCmd.srv"
  "srv/ArmCmd.srv"
  "srv/RealCoorCmd.srv"
  "srv/PublishOoiCmd.srv"
  DEPENDENCIES geometry_msgs
)
ament_package()
