cmake_minimum_required(VERSION 3.0.2)
project(image_editor)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    sensor_msgs
    geometry_msgs
    std_msgs
    image_transport
    cv_bridge
    dynamic_reconfigure
    nodelet
    pluginlib
)

generate_dynamic_reconfigure_options(
    cfg/Parameter.cfg
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

add_library(${PROJECT_NAME} src/image_editor_nodelet.cpp src/image_editor.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_node src/image_editor.cpp src/image_editor_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
