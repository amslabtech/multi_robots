#ifndef MULTI_ROBOTS_MESSAGE_REVISER_H_
#define MULTI_ROBOTS_MESSAGE_REVISER_H_

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_srvs/SetBool.h>

#include "roomba_500driver_meiji/RoombaCtrl.h"

class MessageReviser {
 public:
    MessageReviser();
    void process();
    void roomba_odometry_callback(const nav_msgs::OdometryConstPtr &odom);
    void local_cmd_vel_callback(const geometry_msgs::TwistConstPtr &twist);
    bool reached_goal_service(std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &res);
    roomba_500driver_meiji::RoombaCtrl create_ctrl(double linear_x, double angular_z);

 private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber roomba_odometry_sub_;
    ros::Subscriber local_cmd_vel_sub_;
    ros::Publisher corrected_odom_pub_;
    ros::Publisher roomba_ctrl_pub_;
    ros::ServiceServer reaced_goal_service_;
    bool update_local_cmd_vel_;
    bool reached_goal_;
    int HZ;
    double LINEAR_COEF;
    double START_SPEED;
};

#endif  // MULTI_ROBOTS_MESSAGE_REVISER_H_
