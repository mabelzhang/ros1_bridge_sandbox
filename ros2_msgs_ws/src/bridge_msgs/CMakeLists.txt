cmake_minimum_required(VERSION 3.5)
project(bridge_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(bridge_msgs
   msg/CANFrame.msg
   msg/CurbGo.msg
   msg/JointCommand.msg
   msg/MotorStatus.msg
   #srv/GoToGoalPose.srv
   #srv/TravelSetDistance.srv
   #srv/TravelSetTime.srv
   #srv/AlignRobot.srv
  DEPENDENCIES
   geometry_msgs
   std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()

#install(
#  FILES ros1_bridge_mappings.yaml
#  DESTINATION share/${PROJECT_NAME})
