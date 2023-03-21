#include "roomba_dynamixel_controller/roomba_dynamixel_controller.h"

RoombaDynamixelController::RoombaDynamixelController() : private_nh_("~")
{
    private_nh_.param("DYNAMIXEL_NAME",DYNAMIXEL_NAME_,{std::string("dynamixel")});
    private_nh_.param("DYNAMIXEL_FRAME_ID",DYNAMIXEL_FRAME_ID_,{std::string("dynamixel")});
    private_nh_.param("ROBOT_FRAME_ID",ROBOT_FRAME_ID_,{std::string("base_link")});
    private_nh_.param("OFFSET_ANGLE",OFFSET_ANGLE_,{0.0});
    private_nh_.param("EXECUTION_TIME",EXECUTION_TIME_,{1.0});
    private_nh_.param("DYNAMIXEL_X",DYNAMIXEL_X_,{0.0});
    private_nh_.param("DYNAMIXEL_Y",DYNAMIXEL_Y_,{0.0});
    private_nh_.param("DYNAMIXEL_Z",DYNAMIXEL_Z_,{0.663});
    private_nh_.param("DYNAMIXEL_ROLL",DYNAMIXEL_ROLL_,{0.0});
    private_nh_.param("DYNAMIXEL_PITCH",DYNAMIXEL_PITCH_,{0.0});

    joint_sub_ = nh_.subscribe("joint_in",10,&RoombaDynamixelController::jointstate_callback,this);
    angle_sub_ = nh_.subscribe("angle_in",10,&RoombaDynamixelController::angle_callback,this);

    joint_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("joint_out",1);
    
    private_nh_.param("IS_TF",IS_TF_,{true});
    if(IS_TF_) broadcaster_.reset(new tf2_ros::TransformBroadcaster);
}

void RoombaDynamixelController::angle_callback(const dynamixel_angle_msgs::DynamixelAngle::ConstPtr& msg)
{
    ROS_INFO("has received angle!");
    publish_angle(msg->angle);
}

void RoombaDynamixelController::init_jt_msg(trajectory_msgs::JointTrajectory& jt)
{
    jt.points.resize(1);
    jt.joint_names.resize(1);
    jt.joint_names[0] = DYNAMIXEL_NAME_;
    jt.points[0].positions.resize(2);
}

void RoombaDynamixelController::jointstate_callback(const sensor_msgs::JointState::ConstPtr& msg)
{
    if(IS_TF_){
        geometry_msgs::TransformStamped dynamixel_pose;
        dynamixel_pose.header.stamp = ros::Time::now();
        dynamixel_pose.header.frame_id = ROBOT_FRAME_ID_;
        dynamixel_pose.child_frame_id = DYNAMIXEL_FRAME_ID_;
        dynamixel_pose.transform.translation.x = DYNAMIXEL_X_;
        dynamixel_pose.transform.translation.y = DYNAMIXEL_Y_;
        dynamixel_pose.transform.translation.z = DYNAMIXEL_Z_;

        tf2::Quaternion q;
        q.setRPY(DYNAMIXEL_ROLL_,DYNAMIXEL_PITCH_,msg->position[0]);
        dynamixel_pose.transform.rotation.x = q.x();
        dynamixel_pose.transform.rotation.y = q.y();
        dynamixel_pose.transform.rotation.z = q.z();
        dynamixel_pose.transform.rotation.w = q.w();
        broadcaster_->sendTransform(dynamixel_pose);
    }
}

void RoombaDynamixelController::publish_angle(double angle = 0.0)
{
    normalize_angle(angle);
    trajectory_msgs::JointTrajectory jt;
    init_jt_msg(jt);
    jt.points[0].positions[0] = angle;
    jt.points[0].time_from_start = ros::Duration(EXECUTION_TIME_);
    joint_pub_.publish(jt);
}

void RoombaDynamixelController::normalize_angle(double& angle)
{
    while(angle > M_PI || angle <= -M_PI){
        if(angle > M_PI) angle -= 2*M_PI;
        if(angle < -M_PI) angle += 2*M_PI;
    }
}

void RoombaDynamixelController::offset_angle(double& angle)
{
    if(OFFSET_ANGLE_ > 0){
        if(angle > M_PI - OFFSET_ANGLE_) angle += OFFSET_ANGLE_ - 2*M_PI;
        else angle += OFFSET_ANGLE_;
    }
    else if(OFFSET_ANGLE_ < 0){
        if(angle < -M_PI - OFFSET_ANGLE_) angle = OFFSET_ANGLE_ + 2*M_PI;
        else angle += OFFSET_ANGLE_;
    }
}

void RoombaDynamixelController::process(){ ros::spin();}

int main(int argc,char **argv)
{
    ros::init(argc,argv,"roomba_dynamixel_controller");
    RoombaDynamixelController controller;
    controller.process();
    return 0;
}
