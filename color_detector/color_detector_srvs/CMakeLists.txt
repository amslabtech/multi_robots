cmake_minimum_required(VERSION 3.0.2)
project(color_detector_srvs)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    message_generation
)

add_service_files(
    FILES
    ColorEnable.srv
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(
    CATKIN_DEPENDS roscpp std_msgs message_runtime
)