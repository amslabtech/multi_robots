#ifndef MULTI_ROBOTS_TF_ROOMBA_TF2_BROADCASTER_H_
#define MULTI_ROBOTS_TF_ROOMBA_TF2_BROADCASTER_H_

#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

namespace multi_robots {
class RoombaTf2Broadcaster {
 public:
    RoombaTf2Broadcaster();
    void process();
 private:
    ros::NodeHandle nh;
    ros::NodeHandle private_nh;
    ros::Subscriber odom_sub;
    tf2_ros::TransformBroadcaster broadcaster;

    void odom_callback(const nav_msgs::OdometryConstPtr &odom);
};
}  // namespace multi_robots

#endif  // MULTI_ROBOTS_TF_ROOMBA_TF2_BROADCASTER_H_
