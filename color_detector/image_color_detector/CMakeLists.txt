cmake_minimum_required(VERSION 3.0.2)
project(image_color_detector)

add_compile_options(-O3)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    cv_bridge
    color_detector_msgs
    color_detector_params
)

find_package(OpenCV 4 REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
    src/image_color_detector.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(
    image_color_detector_node src/image_color_detector_node.cpp src/image_color_detector.cpp
)

add_dependencies(
    image_color_detector_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(image_color_detector_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)
