#include "ros/ros.h"

#include "dynamixel_angle_msgs/DynamixelAngle.h"
#include "color_detector_msgs/TargetAngleList.h"

class Example {
 public:
    Example() {
        dynamixel_pub = nh.advertise<dynamixel_angle_msgs::DynamixelAngle>("/angle", 1);
        theta_s_sub = nh.subscribe("/target/angle", 1, &Example::callback, this);
    }
    void callback(const color_detector_msgs::TargetAngleListConstPtr &target_angles) {
        for (const auto &target : target_angles->data) {
            if (target.color != "blue" || target.cluster_num < 500) continue;
            dynamixel_angle_msgs::DynamixelAngle msg;
            msg.theta = target.radian;
            dynamixel_pub.publish(msg);
        }
    }
    void process() { ros::spin(); }
 private:
    ros::NodeHandle nh;
    ros::Publisher dynamixel_pub;
    ros::Subscriber theta_s_sub;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "example_angle_theta");
    Example exam;
    exam.process();
    return 0;
}
