#ifndef IMAGE_EDITOR_H
#define IMAGE_EDITOR_H

#include <ros/ros.h>
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include "sensor_msgs/image_encodings.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/xphoto/white_balance.hpp"

class ImageEditor
{
    public:
        ImageEditor();
        explicit ImageEditor(ros::NodeHandle input_nh, ros::NodeHandle input_private_nh);
        void cv_process();
        void set_threshold(float threshold);
        std::string get_roomba();

    private:
        //method
        void cv_image_callback(const sensor_msgs::ImageConstPtr&);

        //param
        std::string roomba;
        float th;

        //member
        ros::NodeHandle nh;
        ros::NodeHandle private_nh;
        ros::Subscriber image_sub;
        ros::Publisher image_pub;

        sensor_msgs::ImagePtr cv_sub_image;
        sensor_msgs::ImagePtr cv_pub_image;
};

#endif
