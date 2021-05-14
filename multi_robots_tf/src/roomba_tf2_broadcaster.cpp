#include "multi_robots_tf/roomba_tf2_broadcaster.h"

namespace multi_robots {
RoombaTf2Broadcaster::RoombaTf2Broadcaster() : private_nh("~") {
    odom_sub = nh.subscribe("/roomba/odometry", 1, &RoombaTf2Broadcaster::odom_callback, this);
}

void RoombaTf2Broadcaster::odom_callback(const nav_msgs::OdometryConstPtr &odom) {
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header = odom->header;
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = odom->pose.pose.position.x;
    odom_trans.transform.translation.y = odom->pose.pose.position.y;
    odom_trans.transform.translation.z = odom->pose.pose.position.z;
    odom_trans.transform.rotation = odom->pose.pose.orientation;

    broadcaster.sendTransform(odom_trans);
    return;
}

void RoombaTf2Broadcaster::process() {
    ros::spin();
    return;
}
}  // namespace multi_robots

int main(int argc, char **argv) {
    ros::init(argc, argv, "roomba_tf2_broadcaster.h");
    multi_robots::RoombaTf2Broadcaster roomba_tf2_broadcaster;
    roomba_tf2_broadcaster.process();
    return 0;
}
