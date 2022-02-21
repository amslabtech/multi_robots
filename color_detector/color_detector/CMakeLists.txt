cmake_minimum_required(VERSION 3.0.2)
project(color_detector)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    std_msgs
    pluginlib
    nodelet
    dynamic_reconfigure
    point_cloud_color_detector
    image_color_detector
    color_detector_params
    color_detector_msgs
    color_detector_srvs
)

find_package(OpenCV 4 REQUIRED)
find_package(PCL 1.10 REQUIRED)

catkin_package(
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
    src/color_detector_nodelet.cpp
)

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${PCL_LIBRARIES}
)
