cmake_minimum_required(VERSION 3.8)
project(turtlebot3_custom_simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)

set(EXEC_NAME turtlebot3_custom_simulation)

add_executable(${EXEC_NAME} src/turtlebot3_custom_simulation.cpp)
target_compile_features(${EXEC_NAME} PRIVATE cxx_std_17)
target_include_directories(${EXEC_NAME} PRIVATE
  include
)
target_link_libraries(${EXEC_NAME} PRIVATE
  rclcpp::rclcpp
  tf2::tf2
  geometry_msgs::geometry_msgs__rosidl_typesupport_cpp
  nav_msgs::nav_msgs__rosidl_typesupport_cpp
  sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
  tf2_msgs::tf2_msgs__rosidl_typesupport_cpp
)

if(APPLE)
  # Some RoboStack exports over-link every available typesupport and Python
  # generator.  Remove dylibs that this C++ executable does not actually use.
  target_link_options(${EXEC_NAME} PRIVATE LINKER:-dead_strip_dylibs)
  # Derive the runtime search paths from the imported ROS targets.  This keeps
  # RoboStack relocatable and avoids hard-coding Conda or user-specific paths.
  set_target_properties(${EXEC_NAME} PROPERTIES
    INSTALL_RPATH_USE_LINK_PATH TRUE
  )
endif()

install(TARGETS ${EXEC_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch param
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(nav_msgs)
ament_export_dependencies(rclcpp)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(tf2)
ament_export_dependencies(tf2_msgs)
ament_package()
