#include "ros/ros.h"

#include "dynamixel_angle_msgs/DynamixelAngle.h"
#include "color_detector_msgs/TargetAngleList.h"

void callback(const color_detector_msgs::TargetAngleListConstPtr &target_angles, const ros::Publisher &dynamixel_pub) {
    for (const auto &target : target_angles->data) {
        if (target.color != "blue" || target.cluster_num < 100) continue;
        dynamixel_angle_msgs::DynamixelAngle msg;
        msg.theta = target.radian;
        dynamixel_pub.publish(msg);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "example angle theta");
    ros::NodeHandle nh;
    ros::Publisher dynamixel_pub = nh.advertise<dynamixel_angle_msgs::DynamixelAngle>("/angle", 1);
    ros::Subscriber theta_s_sub = nh.subscribe<color_detector_msgs::TargetAngleList>("/target/angle", 1, boost::bind(callback, _1, dynamixel_pub))color;
}
