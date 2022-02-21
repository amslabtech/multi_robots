cmake_minimum_required(VERSION 3.0.2)
project(color_detector_params)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
    cfg/Hsv.cfg
)

catkin_package(
    INCLUDE_DIRS include
    # LIBRARIES ${PROJECT_NAME}
    # CATKIN_DEPENDS roscpp
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)