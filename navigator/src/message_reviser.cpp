#include "navigator/message_reviser.h"

MessageReviser::MessageReviser() : private_nh_("~"), update_local_cmd_vel_(false), reached_goal_(false) {
    roomba_odometry_sub_ = nh_.subscribe("roomba/odometry", 1, &MessageReviser::roomba_odometry_callback, this);
    local_cmd_vel_sub_ = nh_.subscribe("local_path/cmd_vel", 1, &MessageReviser::local_cmd_vel_callback, this);
    corrected_odom_pub_ = nh_.advertise<nav_msgs::Odometry>("roomba/corrected_odometry", 1);
    roomba_ctrl_pub_ = nh_.advertise<roomba_500driver_meiji::RoombaCtrl>("roomba/control", 1);
    reaced_goal_service_ = nh_.advertiseService("reached_goal", &MessageReviser::reached_goal_service, this);
    private_nh_.param("HZ", HZ, 10);
    private_nh_.param("LINEAR_COEF", LINEAR_COEF, 2.0);
    private_nh_.param("START_SPEED", START_SPEED, 0.3);
}

void MessageReviser::roomba_odometry_callback(const nav_msgs::OdometryConstPtr &odom) {
    nav_msgs::Odometry corrected_odom = *odom;
    corrected_odom.twist.twist.linear.x /= LINEAR_COEF;
    corrected_odom.twist.twist.angular.z /= LINEAR_COEF;
    corrected_odom_pub_.publish(corrected_odom);
}

void MessageReviser::local_cmd_vel_callback(const geometry_msgs::TwistConstPtr &twist) {
    if (reached_goal_) return;
    roomba_500driver_meiji::RoombaCtrl roomba_ctrl = create_ctrl(twist->linear.x, twist->angular.z);
    roomba_ctrl_pub_.publish(roomba_ctrl);
    update_local_cmd_vel_ = true;
}

bool MessageReviser::reached_goal_service(std_srvs::SetBoolRequest &req,
                                          std_srvs::SetBoolResponse &res) {
    ROS_INFO_STREAM("Recieved reached_goal call. value = " << (req.data ? "true." : "false."));
    if (req.data) {
        reached_goal_ = true;
        roomba_500driver_meiji::RoombaCtrl roomba_ctrl = create_ctrl(0.0, 0.0);
        roomba_ctrl_pub_.publish(roomba_ctrl);
        res.message = "Set reached_goal = true.";
    } else {
        reached_goal_ = false;
        res.message = "Set reached_goal = false.";
    }
    res.success = true;
    return true;
}

roomba_500driver_meiji::RoombaCtrl MessageReviser::create_ctrl(double linear_x, double angular_z) {
    roomba_500driver_meiji::RoombaCtrl roomba_ctrl;
    roomba_ctrl.mode = roomba_500driver_meiji::RoombaCtrl::DRIVE_DIRECT;
    roomba_ctrl.cntl.linear.x = linear_x;
    roomba_ctrl.cntl.angular.z = angular_z;
    return roomba_ctrl;
}

void MessageReviser::process() {
    ros::Rate loop_rate(HZ);
    while (ros::ok()) {
        if (!update_local_cmd_vel_) {
            ROS_WARN_THROTTLE(10.0, "Stop. Local command velocity is not update");
            roomba_500driver_meiji::RoombaCtrl roomba_ctrl = create_ctrl(0., 0.);
            roomba_ctrl_pub_.publish(roomba_ctrl);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}
