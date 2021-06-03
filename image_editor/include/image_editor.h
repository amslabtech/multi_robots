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

class ImageConverter
{
    public:
        ImageConverter();
        void cv_process();

    private:
        //method
        void cv_image_callback(const sensor_msgs::ImageConstPtr&);
        void white_balance();

        //param
        float th;

        //member
        ros::NodeHandle nh;
        ros::NodeHandle private_nh;
        ros::Subscriber image_sub;
        ros::Publisher image_pub;

        sensor_msgs::ImagePtr cv_sub_image;
        sensor_msgs::ImagePtr cv_pub_image;

};

class ImageEditor
{
    public:
        ImageEditor();
        void process();

    private:
        //method
        void image_callback(const sensor_msgs::ImageConstPtr &msg);
        void image_edit();


        //param
        int row;
        int col;
        int step;

        std::vector<int> raw_data;
        std::vector<int> edited_data;

        //member
        ros::NodeHandle nh;
        ros::NodeHandle private_nh;
        ros::Subscriber sub_image;
        ros::Publisher pub_image;

        sensor_msgs::Image raw_image;
        sensor_msgs::Image edited_image;
};
#endif
