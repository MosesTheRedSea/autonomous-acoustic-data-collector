cmake_minimum_required(VERSION 3.8)
project(audition_data_collector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(audition_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rcutils REQUIRED)
find_package(nav_msgs REQUIRED)

set(dependencies
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  audition_msgs
  rosbag2_cpp
  rosbag2_storage
  nav_msgs
)


add_executable(controller src/controller.cpp)
ament_target_dependencies(controller ${dependencies})

add_executable(waypoint src/waypoint.cpp)
ament_target_dependencies(waypoint ${dependencies})

add_executable(collector src/collector.cpp)
ament_target_dependencies(collector ${dependencies})

add_executable(handler src/handler.cpp)
ament_target_dependencies(handler ${dependencies})

add_executable(recorder src/recorder.cpp)
ament_target_dependencies(recorder ${dependencies})

install(PROGRAMS
  scripts/acoustic_recorder.py
  scripts/odom_integrator.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  waypoint
  collector
  handler
  controller
  recorder
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
