#ifndef AMCL_POSE_REPUBLISHER_H_
#define AMCL_POSE_REPUBLISHER_H_

#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

class AMCLPoseRepublisher {
 public:
    AMCLPoseRepublisher();
    void timer_callback(const ros::TimerEvent&);
    void process();

 private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Publisher pose_pub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    double HZ;
    std::string ROOMBA;
    ros::Timer timer_;
};

#endif  // AMCL_POSE_REPUBLISHER_H_
