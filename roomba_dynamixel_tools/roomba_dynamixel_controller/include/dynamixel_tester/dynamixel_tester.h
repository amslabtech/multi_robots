#ifndef DYNAMIXEL_TESTER_H_
#define DYNAMIXEL_TESTER_H_

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TransformStamped.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/utils.h>

class DynamixelTester
{
public:
    DynamixelTester();
    void process();

private:
    void jointstate_callback(const sensor_msgs::JointState::ConstPtr& msg);
    
    void init_jt_msg(trajectory_msgs::JointTrajectory& jt);
    void set_mode(std::string mode);
    void publish_angle(double angle);
    void normalize_angle(double& angle);

    // node handler
    ros::NodeHandle private_nh_;
    ros::NodeHandle nh_;
    
    // subscriber
    ros::Subscriber joint_sub_;
    
    // publisher
    ros::Publisher joint_pub_;

    // tf
    boost::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;

    // buffer
    std::string mode_;

    // parameter
    std::string DYNAMIXEL_NAME_;
    std::string BASE_LINK_FRAME_ID_;     
    std::string DYNAMIXEL_FRAME_ID_;
    bool IS_TF_;
    double HZ_;
    double TARGET_ANGLE_;
    double DYNAMIXEL_X_;
    double DYNAMIXEL_Y_;
    double DYNAMIXEL_Z_;
    double DYNAMIXEL_ROLL_;
    double DYNAMIXEL_PITCH_;
};

#endif  // DYNAMIXEL_TESTER_H_