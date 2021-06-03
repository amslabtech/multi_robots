#ifndef ROOMBA_VISUALIZER_H_
#define ROOMBA_VISUALIZER_H_

#include <color_detector_msgs/TargetPosition.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

class RoombaVisualizer {
 public:
    RoombaVisualizer();
    void pose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &pose);
    void sync_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &pose,
                       const color_detector_msgs::TargetPositionConstPtr &target);
    visualization_msgs::Marker create_marker(ros::Time time = ros::Time::now(),
                                             std::string roomba = "roomba",
                                             std::string name = "marker", double scale = 0.5,
                                             double x = 0., double y = 0., float r = 1.f,
                                             float g = 0.f, float b = 0.f, float a = 0.6f);
    void process();

 private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber pose_sub_;
    message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> sync_pose_sub_;
    message_filters::Subscriber<color_detector_msgs::TargetPosition> target_position_sub_;

    typedef message_filters::sync_policies::ApproximateTime<
        geometry_msgs::PoseWithCovarianceStamped, color_detector_msgs::TargetPosition>
        sync_policy_;
    message_filters::Synchronizer<sync_policy_> synchronizer;
    ros::Publisher marker_pub_;

    std::string ROOMBA;
};

#endif  // ROOMBA_VISUALIZER_H_
