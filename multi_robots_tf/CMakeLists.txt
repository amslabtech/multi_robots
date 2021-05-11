cmake_minimum_required(VERSION 3.0.2)
project(multi_robots_tf)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roomba_500driver_meiji
  roscpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(static_tf2_broadcaster src/static_tf2_broadcaster.cpp)
target_link_libraries(static_tf2_broadcaster ${catkin_LIBRARIES})
