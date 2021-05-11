#include "multi_robots_tf/static_tf2_broadcaster.h"

namespace multi_robots {
StaticTf2Broadcaster::StaticTf2Broadcaster() : private_nh("~") {
    private_nh.param("base_link_frame_id", base_link_frame_id, std::string("base_link"));
    private_nh.param("lidar_frame_id", lidar_frame_id, std::string("scan"));
    private_nh.param("thetas_frame_id", thetas_frame_id, std::string("theta_s"));
    private_nh.param("realsense_frame_id", realsense_frame_id, std::string("realsense"));
    private_nh.param("lidar_x", lidar_x, 0.0);
    private_nh.param("lidar_y", lidar_y, 0.0);
    private_nh.param("lidar_z", lidar_z, 0.0);
    private_nh.param("lidar_roll", lidar_roll, 0.0);
    private_nh.param("lidar_pitch", lidar_pitch, 0.0);
    private_nh.param("lidar_yaw", lidar_yaw, 0.0);
    private_nh.param("thetas_x", thetas_x, 0.0);
    private_nh.param("thetas_y", thetas_y, 0.0);
    private_nh.param("thetas_z", thetas_z, 0.0);
    private_nh.param("thetas_roll", thetas_roll, 0.0);
    private_nh.param("thetas_pitch", thetas_pitch, 0.0);
    private_nh.param("thetas_yaw", thetas_yaw, 0.0);
    private_nh.param("realsense_x", realsense_x, 0.0);
    private_nh.param("realsense_y", realsense_y, 0.0);
    private_nh.param("realsense_z", realsense_z, 0.0);
    private_nh.param("realsense_roll", realsense_roll, 0.0);
    private_nh.param("realsense_pitch", realsense_pitch, 0.0);
    private_nh.param("realsense_yaw", realsense_yaw, 0.0);

    lidar_transformStamped = create_transformStamped_msg(lidar_frame_id, lidar_x, lidar_y, lidar_z,
                                                         lidar_roll, lidar_pitch, lidar_yaw);
    thetas_transformStamped = create_transformStamped_msg(
        thetas_frame_id, thetas_x, thetas_y, thetas_z, thetas_roll, thetas_pitch, thetas_yaw);
    realsense_transformStamped =
        create_transformStamped_msg(realsense_frame_id, realsense_x, realsense_y, realsense_z,
                                    realsense_roll, realsense_pitch, realsense_yaw);
}

geometry_msgs::TransformStamped StaticTf2Broadcaster::create_transformStamped_msg(
    std::string child_frame_id, double x, double y, double z, double roll, double pitch,
    double yaw) {
    geometry_msgs::TransformStamped static_transformStamped;

    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id = base_link_frame_id;
    static_transformStamped.child_frame_id = child_frame_id;

    static_transformStamped.transform.translation.x = x;
    static_transformStamped.transform.translation.y = y;
    static_transformStamped.transform.translation.z = z;

    tf2::Quaternion quatanion;
    quatanion.setRPY(roll, pitch, yaw);
    static_transformStamped.transform.rotation.x = quatanion.x();
    static_transformStamped.transform.rotation.y = quatanion.y();
    static_transformStamped.transform.rotation.z = quatanion.z();
    static_transformStamped.transform.rotation.w = quatanion.w();

    return static_transformStamped;
}

void StaticTf2Broadcaster::process() {
    static_broadcaster.sendTransform(lidar_transformStamped);
    static_broadcaster.sendTransform(thetas_transformStamped);
    static_broadcaster.sendTransform(realsense_transformStamped);
    ros::spin();
}
}  // namespace multi_robots

int main(int argc, char **argv) {
    ros::init(argc, argv, "multi_robots_static_tf2_broadcaster");
    multi_robots::StaticTf2Broadcaster static_tf2_broadcaster;
    static_tf2_broadcaster.process();
    return 0;
}
