#include "dynamixel_tester/dynamixel_tester.h"

DynamixelTester::DynamixelTester() : private_nh_("~")
{
    private_nh_.param("DYNAMIXEL_NAME",DYNAMIXEL_NAME_,{std::string("dynamixel")});
    private_nh_.param("BASE_LINK_FRAME_ID",BASE_LINK_FRAME_ID_,{std::string("dynamixel")});
    private_nh_.param("DYNAMIXEL_FRAME_ID",DYNAMIXEL_FRAME_ID_,{std::string("base_link")});

    private_nh_.param("HZ",HZ_,{10});
    private_nh_.param("TARGET_ANGLE",TARGET_ANGLE_,{0.0});
    private_nh_.param("DYNAMIXEL_X",DYNAMIXEL_X_,{0.0});
    private_nh_.param("DYNAMIXEL_Y",DYNAMIXEL_Y_,{0.0});
    private_nh_.param("DYNAMIXEL_Z",DYNAMIXEL_Z_,{0.663});
    private_nh_.param("DYNAMIXEL_ROLL",DYNAMIXEL_ROLL_,{0.0});
    private_nh_.param("DYNAMIXEL_PITCH",DYNAMIXEL_PITCH_,{0.0});

    joint_sub_ = nh_.subscribe("joint_in",1,&DynamixelTester::jointstate_callback,this);
    joint_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("joint_out",1);

    private_nh_.param("IS_TF",IS_TF_,{true});
    if(IS_TF_) broadcaster_.reset(new tf2_ros::TransformBroadcaster);

    std::cout << "========== Parameters ========== " << std::endl;
    std::cout << "HZ : " << HZ_ << std::endl;
    std::cout << "TARGET_ANGLE : " << TARGET_ANGLE_ << std::endl;
    std::cout << "DYNAMIXEL_NAME : " << DYNAMIXEL_NAME_ << std::endl;
    std::string mode;
    private_nh_.param("MODE",mode,{std::string("reader")});
    set_mode(mode);
    std::cout << std::endl;
}

void DynamixelTester::jointstate_callback(const sensor_msgs::JointState::ConstPtr& msg)
{
    if(mode_ == "reader"){
        ROS_INFO("Joint[%s]: (%f,%f,%f)", msg->name[0].c_str(), msg->position[0], msg->velocity[0], msg->effort[0]);
    }
    else if(mode_ == "writer"){
        ROS_INFO("Joint[%s]: %f", msg->name[0].c_str() , msg->position[0]);
    }

    if(IS_TF_){
        geometry_msgs::TransformStamped dynamixel_pose;
        dynamixel_pose.header.stamp = ros::Time::now();
        dynamixel_pose.header.frame_id = BASE_LINK_FRAME_ID_;
        dynamixel_pose.child_frame_id = DYNAMIXEL_FRAME_ID_;
        dynamixel_pose.transform.translation.x = DYNAMIXEL_X_;
        dynamixel_pose.transform.translation.y = DYNAMIXEL_Y_;
        dynamixel_pose.transform.translation.z = DYNAMIXEL_Y_;

        tf2::Quaternion tf_q;
        tf_q.setRPY(DYNAMIXEL_ROLL_,DYNAMIXEL_PITCH_,msg->position[0]);
        dynamixel_pose.transform.rotation.x = tf_q.x();
        dynamixel_pose.transform.rotation.y = tf_q.y();
        dynamixel_pose.transform.rotation.z = tf_q.z();
        dynamixel_pose.transform.rotation.w = tf_q.w();
        broadcaster_->sendTransform(dynamixel_pose);
    }
}

void DynamixelTester::init_jt_msg(trajectory_msgs::JointTrajectory& jt)
{
    jt.header.frame_id = DYNAMIXEL_FRAME_ID_;
    jt.points.resize(1);
    jt.joint_names.resize(1);
    jt.joint_names[0] = DYNAMIXEL_NAME_;
    jt.points[0].positions.resize(2);
    jt.points[0].positions[0] = 0.0;
}

void DynamixelTester::set_mode(std::string mode)
{
    std::cout << "========== MODE ==========" << std::endl;
    if(mode == "reader"){
        std::cout << "MODE: " << mode.c_str() << std::endl << std::endl;;
        std::cout << "start dynamixel reader" << std::endl;
        std::cout << "Joint[name]: (position,velocity,effort)" << std::endl;
        mode_ = mode;
    }
    else if(mode == "writer"){
        std::cout << "MODE: " << mode.c_str() << std::endl << std::endl;;
        std::cout << "start dynamixel writer" << std::endl;
        std::cout << "Joint[name]: position" << std::endl;
        mode_ = mode;
    }
    else{
        std::cerr << "No applicable mode: " << mode_.c_str() << std::endl << std::endl;
        std::cerr << "Please select 'reader' or 'writer'" << std::endl;
        std::cout << "Set 'reader" << std::endl;
        mode_ = std::string("reader");
    }
}

void DynamixelTester::publish_angle(double angle)
{
    normalize_angle(angle);
    trajectory_msgs::JointTrajectory jt;
    init_jt_msg(jt);
    jt.points[0].positions[0] = angle;
    jt.points[0].time_from_start = ros::Duration(1.0);
    joint_pub_.publish(jt);
}

void DynamixelTester::normalize_angle(double& angle)
{
    while(angle > M_PI || angle <= -M_PI){
        if(angle > M_PI) angle -= 2*M_PI;
        if(angle < -M_PI) angle += 2*M_PI;
    }
}

void DynamixelTester::process()
{
    ros::Rate rate(HZ_);
    while(ros::ok()){
        if(mode_ == "writer"){
            publish_angle(0.0);
            ROS_INFO("Wait 5 seconds");
            ros::Duration(5.0);
            publish_angle(TARGET_ANGLE_);
            break;
        }
        ros::spinOnce();
        rate.sleep();
    }
}

int main(int argc,char **argv)
{
    ros::init(argc,argv,"dynamixel_tester");
    DynamixelTester dynamixel_tester;
    dynamixel_tester.process();
    return 0;
}
