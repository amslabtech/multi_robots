#include "navigator/rviz_waypoints_creater.h"

RvizWaypointsCreater::RvizWaypointsCreater() : private_nh_("~") {
    bool only_see;
    private_nh_.param("CONFIRM_WAYPOINTS", only_see, false);
    if (only_see) {
        read_waypoints(&points_);
        ros::Duration(1.0).sleep();  // wait for rviz to start
        timer = nh_.createTimer(ros::Duration(1.0), &RvizWaypointsCreater::timer_callback, this);
        pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("local_goal", 1);
    } else {
        ROS_ASSERT(private_nh_.getParam("filename", filename));
        ROS_INFO_STREAM("output file is " << filename);
        pose_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &RvizWaypointsCreater::pose_callback, this);
    }
}

void RvizWaypointsCreater::pose_callback(const geometry_msgs::PoseStampedConstPtr &pose) {
    static std::ofstream ofs(filename);
    static int index = 0;
    ofs << "waypoint" << index++ << ": [";
    ofs << std::fixed << std::setprecision(14) << pose->pose.position.x << ", " << pose->pose.position.y << ", "
        << pose->pose.position.z << ", " << pose->pose.orientation.w << ", " << pose->pose.orientation.x << ", "
        << pose->pose.orientation.y << ", " << pose->pose.orientation.z << "]\n";
}

void RvizWaypointsCreater::read_waypoints(std::vector<geometry_msgs::PoseStamped> *waypoints) {
    XmlRpc::XmlRpcValue param_list;
    for (size_t i = 0;; i++) {
        std::string param_name = "waypoint" + std::to_string(i);
        if (!private_nh_.getParam(param_name.c_str(), param_list)) break;
        ROS_ASSERT(param_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

        ROS_ASSERT(param_list.size() == 7);
        for (size_t j = 0; j < param_list.size(); j++) {
            ROS_ASSERT(param_list[j].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        }

        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = "map";
        pose_stamped.pose.position.x = param_list[0];
        pose_stamped.pose.position.y = param_list[1];
        pose_stamped.pose.position.z = param_list[2];
        pose_stamped.pose.orientation.w = param_list[3];
        pose_stamped.pose.orientation.x = param_list[4];
        pose_stamped.pose.orientation.y = param_list[5];
        pose_stamped.pose.orientation.z = param_list[6];

        waypoints->push_back(pose_stamped);
    }
    ROS_INFO_STREAM("size = " << waypoints->size());
}

void RvizWaypointsCreater::timer_callback(const ros::TimerEvent &event) {
    static int i = 0;
    if (i >= points_.size()) {
        ros::shutdown();
        return;
    }
    pose_pub_.publish(points_[i]);
    ROS_INFO_STREAM("i = " << i++);
}

void RvizWaypointsCreater::process() { ros::spin(); }

int main(int argc, char **argv) {
    ros::init(argc, argv, "rviz_waypoints_creater");
    RvizWaypointsCreater rviz_waypoints_creater;
    rviz_waypoints_creater.process();
    return 0;
}
