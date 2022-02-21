#ifndef IMAGE_COLOR_DETECTOR_H_
#define IMAGE_COLOR_DETECTOR_H_

#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <opencv2/opencv.hpp>

#include "color_detector_msgs/TargetAngleList.h"
#include "color_detector_params/hsv.h"

using color_detector_params_hsv::HSV;
using color_detector_params_hsv::ThresholdHSV;

class ImageColorDetector {
 public:
    ImageColorDetector();
    explicit ImageColorDetector(ros::NodeHandle nh, ros::NodeHandle private_nh);
    void set_hsv_params();
    void set_image_pubs();
    void image_callback(const sensor_msgs::ImageConstPtr &received_image);
    void to_cv_image(const sensor_msgs::Image &ros_image, cv::Mat &output_image);
    void filter_hsv(const cv::Mat &hsv_image, const ThresholdHSV &thres_hsv, cv::Mat &output_image);
    void detect_target(const cv::Mat &binarized_image, int mag, cv::Mat &output_image,
                       std::vector<std::pair<int, int>> &output_pixels);
    void form_cluster(int start_x, int start_y, int mag, const cv::Mat &mat, std::vector<std::vector<bool>> &reached,
                      std::vector<std::pair<int, int>> &output_pixels);
    void create_target_msg(std::string color, int width, int mag, const std::vector<std::pair<int, int>> &pixels,
                           color_detector_msgs::TargetAngle &output_msg);
    void create_target_image(const std_msgs::Header &header, const cv::Mat &bgr_image,
                             const std::vector<std::pair<int, int>> &pixels, sensor_msgs::ImagePtr &output_msg);
    double calc_angle(double target_position, int width);
    void process();
    std::vector<std::string> colors_;
    std::vector<ThresholdHSV> param_hsvs_;
    bool only_publish_mask_image_;
    bool publish_target_image_;

 private:
    std::string ROOMBA;
    int MIN_CLUSTER_SIZE;
    int MAX_CLUSTER_SIZE;

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    ros::Subscriber image_sub_;
    ros::Publisher target_angle_list_pub_;
    std::vector<ros::Publisher> masked_image_pubs_;
    std::vector<ros::Publisher> target_image_pubs_;
};

#endif  // IMAGE_COLOR_DETECTOR_H_
