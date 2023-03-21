#ifndef ROOMBA_DYNAMIXEL_CONTROLLER_H_
#define ROOMBA_DYNAMIXEL_CONTROLLER_H_

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/utils.h>

// Custom msg
#include "dynamixel_angle_msgs/DynamixelAngle.h"

class RoombaDynamixelController{
public:
    RoombaDynamixelController();
    void process();

private:
    void angle_callback(const dynamixel_angle_msgs::DynamixelAngle::ConstPtr& msg);
    void jointstate_callback(const sensor_msgs::JointState::ConstPtr& msg);

    void init_jt_msg(trajectory_msgs::JointTrajectory& jt);
    void publish_angle(double angle);
    void normalize_angle(double& angle);
    void offset_angle(double& angle);

    // node handle
    ros::NodeHandle private_nh_;
    ros::NodeHandle nh_;

    // publisher
    ros::Publisher joint_pub_;

    // subscriber
    ros::Subscriber angle_sub_;
    ros::Subscriber joint_sub_;

    // tf
    boost::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;

    // params
    bool IS_TF_;
    std::string DYNAMIXEL_NAME_;
    std::string DYNAMIXEL_FRAME_ID_;
    std::string ROBOT_FRAME_ID_;
    double OFFSET_ANGLE_;   // offset angle(no use)
    double EXECUTION_TIME_; // execution time
    double DYNAMIXEL_X_;
    double DYNAMIXEL_Y_;
    double DYNAMIXEL_Z_;
    double DYNAMIXEL_ROLL_;
    double DYNAMIXEL_PITCH_;
};

#endif // ROOMBA_DYNAMIXEL_CONTROLLER_H_
