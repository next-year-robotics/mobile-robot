cmake_minimum_required(VERSION 3.8)
project(mr_aruco_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MarkerTrack.msg"
  "action/AlignToMarker.action"
  DEPENDENCIES std_msgs builtin_interfaces
)

ament_package()
