#include "navigator/target_follower.h"

TargetFollower::TargetFollower() : private_nh_("~"), reached_goal_(false) {
    ROS_ASSERT(private_nh_.getParam("TARGET_ROOMBA", TARGET_ROOMBA));
    private_nh_.param("HZ", HZ, 10.0);
    private_nh_.param("GOAL_THRESHOLD", GOAL_THRESHOLD, 1.0);
    pose_sub_ = nh_.subscribe("amcl_pose", 1, &TargetFollower::pose_callback, this);
    std::string target_pose_topic = "/" + TARGET_ROOMBA + "/amcl_pose";
    target_pose_sub_ = nh_.subscribe(target_pose_topic.c_str(), 1, &TargetFollower::target_pose_callback, this);
    local_goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("local_goal", 1);
    reached_goal_client_ = nh_.serviceClient<std_srvs::SetBool>(nh_.getNamespace() + "/reached_goal");
}

void TargetFollower::pose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose_ptr) {
    pose_ = *pose_ptr;
    return;
}

void TargetFollower::target_pose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& target_pose_ptr) {
    static geometry_msgs::PoseWithCovarianceStamped pre_pose;
    static auto call_reached_goal = [](ros::ServiceClient& client, bool reached_goal) -> void {
        std_srvs::SetBool set_bool;
        set_bool.request.data = reached_goal;
        if (client.call(set_bool)) {
            ROS_DEBUG_STREAM(set_bool.response.message);
        } else {
            ROS_ERROR("Failed to call service reached_goal");
        }
    };
    if (!reached_goal_ && is_close_points(pose_, *target_pose_ptr)) {
        call_reached_goal(reached_goal_client_, true);
        reached_goal_ = true;
        return;
    } else if (reached_goal_ && !is_close_points(pose_, *target_pose_ptr)) {
        call_reached_goal(reached_goal_client_, false);
        reached_goal_ = false;
    } else if (reached_goal_) {
        return;
    } else if (pre_pose == pose_) {
        return;
    }
    geometry_msgs::PoseStamped local_goal;
    local_goal.header = target_pose_ptr->header;
    local_goal.pose = target_pose_ptr->pose.pose;
    local_goal_pub_.publish(local_goal);
    pre_pose = pose_;
}

bool TargetFollower::is_close_points(const geometry_msgs::PoseWithCovarianceStamped& pose,
                                     const geometry_msgs::PoseWithCovarianceStamped& target_pose) {
    double diff_x = pose.pose.pose.position.x - target_pose.pose.pose.position.x;
    double diff_y = pose.pose.pose.position.y - target_pose.pose.pose.position.y;
    if (std::sqrt(diff_x * diff_x + diff_y * diff_y) <= GOAL_THRESHOLD) return true;
    return false;
}

void TargetFollower::process() { ros::spin(); }
