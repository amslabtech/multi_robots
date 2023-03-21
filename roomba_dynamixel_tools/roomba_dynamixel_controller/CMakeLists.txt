cmake_minimum_required(VERSION 3.0.2)
project(roomba_dynamixel_controller)

add_compile_options(-o2 -g -Wall)

find_package(catkin REQUIRED COMPONENTS
  dynamixel_angle_msgs
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf2_ros
  trajectory_msgs
)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# roomba_dynamixel_controller
add_executable(roomba_dynamixel_controller src/roomba_dynamixel_controller.cpp)
target_link_libraries(roomba_dynamixel_controller ${catkin_LIBRARIES})

# dynamixel_tester
add_executable(dynamixel_tester src/dynamixel_tester.cpp)
target_link_libraries(dynamixel_tester ${catkin_LIBRARIES})

# teleop_dynamixel_angle
add_executable(teleop_dynamixel_angle src/teleop_dynamixel_angle.cpp)
target_link_libraries(teleop_dynamixel_angle ${catkin_LIBRARIES})
