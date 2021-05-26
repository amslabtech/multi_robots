#include "roomba_visualizer/roomba_visualizer.h"

RoombaVisualizer::RoombaVisualizer()
    : private_nh_("~"),
      sync_pose_sub_(nh_, "amcl_pose", 10),
      target_position_sub_(nh_, "target/position", 10),
      synchronizer(sync_policy_(10), sync_pose_sub_, target_position_sub_) {
    pose_sub_ = nh_.subscribe("amcl_pose", 1, &RoombaVisualizer::pose_callback, this);
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("marker", 1);
    synchronizer.registerCallback(boost::bind(&RoombaVisualizer::sync_callback, this, _1, _2));

    private_nh_.param("ROOMBA", ROOMBA, std::string("roomba"));
}

void RoombaVisualizer::pose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &pose) {
    auto pose_marker = create_marker(pose->header.stamp, ROOMBA);
    marker_pub_.publish(pose_marker);
}

void RoombaVisualizer::sync_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &pose,
                                const color_detector_msgs::TargetPositionConstPtr &target) {
    auto target_marker = create_marker(target->header.stamp, ROOMBA, "measured", 0.3, target->z,
                                       -target->x, 0.f, 1.f, 0.f, 1.f);
    marker_pub_.publish(target_marker);
    return;
}

visualization_msgs::Marker RoombaVisualizer::create_marker(ros::Time time, std::string roomba,
                                                           std::string name, double scale, double x,
                                                           double y, float r, float g, float b,
                                                           float a) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = roomba + "/base_link";
    marker.header.stamp = time;
    marker.ns = roomba + "/" + name;
    marker.id = 0;

    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration();

    marker.scale.x = scale;
    marker.scale.y = scale;
    marker.scale.z = 0.2;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0;
    marker.pose.orientation.w = 1;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;

    return marker;
}

void RoombaVisualizer::process() {
    ros::spin();
    return;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "roomba_visualizer");
    RoombaVisualizer roomba_visualizer;
    roomba_visualizer.process();
    return 0;
}
