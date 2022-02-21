#ifndef MULTI_ROBOTS_TARGET_FOLLOWER_H_
#define MULTI_ROBOTS_TARGET_FOLLOWER_H_

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <std_srvs/SetBool.h>

class TargetFollower {
 public:
    TargetFollower();
    void process();
    void pose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &pose_ptr);
    void target_pose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &target_pose_ptr);
    bool is_close_points(const geometry_msgs::PoseWithCovarianceStamped &pose,
                         const geometry_msgs::PoseWithCovarianceStamped &target_pose);

 private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber pose_sub_;
    ros::Subscriber target_pose_sub_;
    ros::Publisher local_goal_pub_;
    ros::ServiceClient reached_goal_client_;
    std::string TARGET_ROOMBA;
    double HZ;
    double GOAL_THRESHOLD;
    geometry_msgs::PoseWithCovarianceStamped pose_;
    bool reached_goal_;
};

#endif  // MULTI_ROBOTS_TARGET_FOLLOWER_H_
