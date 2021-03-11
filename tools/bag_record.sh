#!/bin/bash

if [ $# -ne 1 ] ; then
    echo "Usage: $0 [bagfile_name]";
    exit 1
fi

rosparam set /camera/aligned_depth_to_color/image_raw/compressed/format png
rosparam set /camera/aligned_depth_to_color/image_raw/compressed/png_level 6

rosbag record /camera/aligned_depth_to_color/image_raw/compressed /camera/color/camera_info /camera/color/image_rect_color/compressed -o $1

