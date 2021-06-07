cmake_minimum_required(VERSION 3.0.2)
project(image_editor)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    geometry_msgs
    std_msgs
    image_transport
    cv_bridge
)

catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES first_challenge_marika
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_executable(image_editor src/image_editor.cpp)
target_link_libraries(image_editor ${catkin_LIBRARIES})
