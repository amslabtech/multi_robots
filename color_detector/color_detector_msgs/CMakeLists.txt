cmake_minimum_required(VERSION 3.0.2)
project(color_detector_msgs)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    message_generation
)

add_message_files(
    FILES
    TargetPosition.msg
    TargetAngle.msg
    TargetAngleList.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(
    CATKIN_DEPENDS roscpp std_msgs message_runtime
)