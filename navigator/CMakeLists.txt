cmake_minimum_required(VERSION 3.0.2)
project(navigator)

find_package(catkin REQUIRED COMPONENTS
  amcl
  std_msgs
  std_srvs
  nav_msgs
  dwa_planner
  geometry_msgs
  roomba_500driver_meiji
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(waypoints_manager_node src/waypoints_manager_node.cpp src/waypoints_manager.cpp)
target_link_libraries(waypoints_manager_node ${catkin_LIBRARIES})

add_executable(message_reviser_node src/message_reviser.cpp src/message_reviser_node.cpp)
target_link_libraries(message_reviser_node ${catkin_LIBRARIES})

add_executable(rviz_waypoints_creater src/rviz_waypoints_creater.cpp)
target_link_libraries(rviz_waypoints_creater ${catkin_LIBRARIES})

add_executable(target_follower src/target_follower.cpp src/target_follower_node.cpp)
target_link_libraries(target_follower ${catkin_LIBRARIES})

add_executable(amcl_pose_republisher src/amcl_pose_republisher.cpp)
target_link_libraries(amcl_pose_republisher ${catkin_LIBRARIES})
