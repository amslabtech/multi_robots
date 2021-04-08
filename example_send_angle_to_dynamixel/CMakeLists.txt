cmake_minimum_required(VERSION 3.0.2)
project(example_send_angle_to_dynamixel)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    color_detector_msgs
    dynamixel_angle_msgs
)

catkin_package()

include_directories(
    ${catkin_INCLUDE_DIRS}
)

add_executable(follow_blue src/follow_blue.cpp)
add_dependencies(follow_blue ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(follow_blue ${catkin_LIBRARIES})
