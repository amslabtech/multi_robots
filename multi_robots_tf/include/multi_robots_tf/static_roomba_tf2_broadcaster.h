#ifndef MULTI_ROBOTS_TF_STATIC_ROOMBA_TF2_BROADCASTER_H_
#define MULTI_ROBOTS_TF_STATIC_ROOMBA_TF2_BROADCASTER_H_

#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>

namespace multi_robots {
class StaticTf2Broadcaster {
 public:
    StaticTf2Broadcaster();
    void process();

 private:
    ros::NodeHandle private_nh;
    std::string base_link_frame_id;
    std::string lidar_frame_id;
    std::string thetas_frame_id;
    std::string realsense_frame_id;
    double lidar_x;
    double lidar_y;
    double lidar_z;
    double lidar_roll;
    double lidar_pitch;
    double lidar_yaw;
    double thetas_x;
    double thetas_y;
    double thetas_z;
    double thetas_roll;
    double thetas_pitch;
    double thetas_yaw;
    double realsense_x;
    double realsense_y;
    double realsense_z;
    double realsense_roll;
    double realsense_pitch;
    double realsense_yaw;

    tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped lidar_transformStamped;
    geometry_msgs::TransformStamped thetas_transformStamped;
    geometry_msgs::TransformStamped realsense_transformStamped;

    geometry_msgs::TransformStamped create_transformStamped_msg(std::string child_frame_id,
                                                                double x, double y, double z,
                                                                double roll, double pitch,
                                                                double yaw);
};
}  // namespace multi_robots
#endif  // MULTI_ROBOTS_TF_STATIC_ROOMBA_TF2_BROADCASTER_H_
