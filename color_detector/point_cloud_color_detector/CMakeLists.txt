cmake_minimum_required(VERSION 3.0.2)
project(point_cloud_color_detector)

add_compile_options(-O3)
add_compile_options(-mtune=native)
add_compile_options(-march=native)
add_compile_options(-mfpmath=both)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    pcl_ros
    color_detector_msgs
    color_detector_srvs
    color_detector_params
)

find_package(PCL 1.10 REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
    src/point_cloud_color_detector.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(
    point_cloud_color_detector_node src/point_cloud_color_detector_node.cpp src/point_cloud_color_detector.cpp
)

add_dependencies(
    point_cloud_color_detector_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(point_cloud_color_detector_node
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)

add_executable(
    teleop_color_enable tools/teleop_color_enable.cpp
)

add_dependencies(
    teleop_color_enable ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(teleop_color_enable
    ${catkin_LIBRARIES}
)
