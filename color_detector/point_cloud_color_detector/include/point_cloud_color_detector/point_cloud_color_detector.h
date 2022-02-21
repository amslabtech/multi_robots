#ifndef POINT_CLOUD_COLOR_DETECTOR_H_
#define POINT_CLOUD_COLOR_DETECTOR_H_

#include <pcl/point_types_conversion.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>

#include <string>
#include <vector>

#include "color_detector_msgs/TargetPosition.h"
#include "color_detector_params/HsvConfig.h"
#include "color_detector_params/hsv.h"
#include "color_detector_srvs/ColorEnable.h"

using color_detector_params_hsv::HSV;
using color_detector_params_hsv::ThresholdHSV;

class PointCloudColorDetector {
 public:
    PointCloudColorDetector() : PointCloudColorDetector(ros::NodeHandle(), ros::NodeHandle("~")) {}
    explicit PointCloudColorDetector(ros::NodeHandle nh, ros::NodeHandle private_nh);
    void set_hsv_params();
    void set_color_enable_param();
    void print_all_params();
    bool enable_color(color_detector_srvs::ColorEnable::Request &req, color_detector_srvs::ColorEnable::Response &res);
    void sensor_callback(const sensor_msgs::PointCloud2ConstPtr &received_pc);
    void mask_point_cloud(const ThresholdHSV &thres_hsv, const pcl::PointCloud<pcl::PointXYZRGB> &pc,
                          const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &output);
    void reduce_point_cloud(int mag, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pc);
    void publish_points(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &pc, const std_msgs::Header &header,
                        const ros::Publisher &publisher);
    void detect_target_cluster(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &masked_pc,
                               const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &target_pc);
    void euclidean_clustering(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &pc,
                              std::vector<pcl::PointIndices> &output);
    void get_target_cluster(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &pc,
                            std::vector<pcl::PointIndices> &pc_indices,
                            const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &output);
    color_detector_msgs::TargetPosition calc_target_position(int mag,
                                                             const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &pc);
    void save_csv(const color_detector_msgs::TargetPosition::ConstPtr &target_position);
    void read_target_roombas();
    void process();
    std::vector<std::string> colors_;
    std::map<int, bool> target_roombas_;
    std::vector<ThresholdHSV> param_hsvs_;
    std::vector<bool> use_colors_;
    bool only_publish_mask_points_;
    bool publish_target_points_;
    double HIGHEST_TARGET_Y;
    double LOWEREST_TARGET_Y;

 private:
    double TOLERANCE;
    int MIN_CLUSTER_SIZE;
    int MAX_CLUSTER_SIZE;

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    ros::Subscriber pc_sub_;
    ros::Publisher target_position_pub_;
    std::vector<ros::Publisher> masked_pc_pubs_;
    std::vector<ros::Publisher> target_pc_pubs_;
    ros::ServiceServer color_enable_srv_;
};

#endif  // POINT_CLOUD_COLOR_DETECTOR_H_
