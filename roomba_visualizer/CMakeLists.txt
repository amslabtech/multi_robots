cmake_minimum_required(VERSION 3.0.2)
project(roomba_visualizer)

find_package(catkin REQUIRED COMPONENTS
  color_detector_msgs
  geometry_msgs
  roscpp
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
  message_filters
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES roomba_visualizer
#  CATKIN_DEPENDS color_detector_msgs geometry_msgs roscpp std_msgs tf2 tf2_ros visualization_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(pose_diff_node src/pose_diff_calculator.cpp)
target_link_libraries(pose_diff_node
  ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_node src/roomba_visualizer.cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
