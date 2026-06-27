cmake_minimum_required(VERSION 3.8)
project(tp_slam_interfaces)

# ROS 2 Humble's Python generators still use the FindPythonInterp and
# FindPythonLibs modules removed by CMP0148's NEW behavior in CMake 3.27.
if(POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LandmarkObservation.msg"
  "msg/LandmarkObservationArray.msg"
  "msg/VisualObservability.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
