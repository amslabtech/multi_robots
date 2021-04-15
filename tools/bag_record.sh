#!/bin/bash

if [ $# -ne 1 ] ; then
    echo "Usage: $0 [bagfile_name]";
    exit 1
fi

rosbag record /camera/aligned_depth_to_color/image_raw/compressed /camera/color/camera_info /camera/color/image_rect_color/compressed /theta_s/image_raw/compressed /roomba/odometry /scan -o $1

