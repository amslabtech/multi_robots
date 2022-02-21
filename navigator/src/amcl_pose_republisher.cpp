#include "navigator/amcl_pose_republisher.h"

AMCLPoseRepublisher::AMCLPoseRepublisher()
    : private_nh_("~"), tf_listener_(tf_buffer_) {
    private_nh_.param("HZ", HZ, 10.0);
    private_nh_.getParam("ROOMBA", ROOMBA);
    pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose", 1);
}

void AMCLPoseRepublisher::timer_callback(const ros::TimerEvent &event) {
    geometry_msgs::TransformStamped transform_stamped;
    try {
        transform_stamped = tf_buffer_.lookupTransform("map", ROOMBA + "/base_link", ros::Time(0));
    } catch (tf2::TransformException &ex) {
        ROS_WARN_STREAM(ex.what());
        ros::Duration(1.0).sleep();
        return;
    }
    geometry_msgs::PoseStamped pose, pose_on_map;
    pose.header.frame_id = ROOMBA + "/base_link";
    pose.header.stamp = ros::Time::now();
    pose.pose.orientation.w = 1.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    tf2::doTransform(pose, pose_on_map, transform_stamped);
    geometry_msgs::PoseWithCovarianceStamped msg;
    msg.header = pose_on_map.header;
    msg.pose.pose = pose_on_map.pose;
    pose_pub_.publish(msg);
}

void AMCLPoseRepublisher::process() {
    timer_ = nh_.createTimer(ros::Duration(1.0 / HZ), &AMCLPoseRepublisher::timer_callback, this);
    ros::spin();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "camera_direction_deteminator");
    AMCLPoseRepublisher cdd;
    cdd.process();
    return 0;
}
