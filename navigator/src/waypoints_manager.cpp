#include "navigator/waypoints_manager.h"

WaypointsManager::WaypointsManager()
    : private_nh_("~"), reached_goal_(false), mt_(std::random_device{}()), dist_(0, 1000) {
    pose_sub_ = nh_.subscribe("amcl_pose", 1, &WaypointsManager::pose_callback, this);
    local_goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("local_goal", 1);
    reached_goal_client_ = nh_.serviceClient<std_srvs::SetBool>(nh_.getNamespace() + "/reached_goal");
    private_nh_.param("HZ", HZ, 10);
    private_nh_.param("LOOP_WAYPOINTS", LOOP_WAYPOINTS, false);
    private_nh_.param("RANDOM_WAYPOINTS", RANDOM_WAYPOINTS, false);
    private_nh_.param("EXPORT_WAYPOINTS_SIZE", EXPORT_WAYPOINTS_SIZE, -1);
    private_nh_.param("GOAL_THRESHOLD", GOAL_THRESHOLD, 0.8);
    private_nh_.param("ADVANCE_LENGTH", ADVANCE_LENGTH, 3.0);
    private_nh_.param("RIGHT_DISTANCE", RIGHT_DISTANCE, 0.7);
    bool WITH_RVIZ;
    private_nh_.param("WITH_RVIZ", WITH_RVIZ, false);
    if (RANDOM_WAYPOINTS) {
        read_waypoints(&points_);
        read_points_relation();
        int idx;
        ROS_ASSERT(private_nh_.getParam("FIRST_POSITION", idx));
        pre_pos_idx_ = idx;
        ROS_ASSERT(private_nh_.getParam("FIRST_LOCAL_GOAL", idx));
        next_pos_idx_ = idx;
        waypoints_.push_back(points_[pre_pos_idx_]);
        waypoints_.push_back(points_[next_pos_idx_]);
        if (EXPORT_WAYPOINTS_SIZE > 0) {
            random_waypoint_to_yaml(EXPORT_WAYPOINTS_SIZE);
            ros::shutdown();
            return;
        }
    } else if (!WITH_RVIZ) {
        read_waypoints(&waypoints_);
        ROS_INFO_STREAM("Read waypoints size = " << waypoints_.size() << ".");
    } else {
        local_goal_sub_ = nh_.subscribe("rviz/local_goal", 1, &WaypointsManager::rviz_local_goal_callback, this);
        clear_waypoints_server_ =
            nh_.advertiseService("clear_waypoints", &WaypointsManager::clear_waypoints_service, this);
    }
}

void WaypointsManager::random_waypoint_to_yaml(int size) {
    for (int i = waypoints_.size(); i < size; i++) {
        const auto &candidate = points_relation_[next_pos_idx_];
        int random_idx = dist_(mt_) % candidate.size();
        if (candidate[random_idx] == pre_pos_idx_) {
            random_idx++;
            random_idx %= candidate.size();
        }
        pre_pos_idx_ = next_pos_idx_;
        next_pos_idx_ = candidate[random_idx];
        waypoints_.push_back(points_[next_pos_idx_]);
    }
    ROS_INFO_STREAM("random waypoints size = " << waypoints_.size());

    std::string fname;
    private_nh_.param("WAYPOINTS_FILENAME", fname, std::string(""));
    std::ofstream ofs(fname);
    for (int i = 0; i < waypoints_.size(); i++) {
        ofs << "waypoint" << i << ": [";
        ofs << std::fixed << std::setprecision(14) << waypoints_[i].pose.position.x << ", "
            << waypoints_[i].pose.position.y << ", " << waypoints_[i].pose.position.z << ", "
            << waypoints_[i].pose.orientation.w << ", " << waypoints_[i].pose.orientation.x << ", "
            << waypoints_[i].pose.orientation.y << ", " << waypoints_[i].pose.orientation.z << "]\n";
    }
    ROS_INFO_STREAM("random waypoints file is " << fname);
}

void WaypointsManager::read_waypoints(std::vector<geometry_msgs::PoseStamped> *waypoints) {
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
}

void WaypointsManager::read_points_relation() {
    XmlRpc::XmlRpcValue param_list;
    for (size_t i = 0;; i++) {
        std::string param_name = "relation" + std::to_string(i);
        if (!private_nh_.getParam(param_name.c_str(), param_list)) break;
        ROS_ASSERT(param_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

        std::vector<int> points;
        for (size_t j = 0; j < param_list.size(); j++) {
            ROS_ASSERT(param_list[j].getType() == XmlRpc::XmlRpcValue::TypeInt);
            points.push_back(param_list[j]);
        }
        ROS_ASSERT(points_relation_.size() == i);
        points_relation_.push_back(points);
    }
}

void WaypointsManager::rviz_local_goal_callback(const geometry_msgs::PoseStampedConstPtr &pose) {
    waypoints_.push_back(*pose);
    ROS_INFO_STREAM("Add waypoints. The Number of waypoints is " << waypoints_.size() << ".");
}

bool WaypointsManager::clear_waypoints_service(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res) {
    waypoints_.clear();
    res.success = true;
    res.message = "Clear waypoints.";
    ROS_INFO_STREAM("Clear waypoints.");
    return true;
}

void WaypointsManager::timer_callback(const ros::TimerEvent &event) {
    static size_t way_points_idx = 0;
    static auto call_reached_goal = [](ros::ServiceClient &client, bool reached_goal) -> void {
        std_srvs::SetBool set_bool;
        set_bool.request.data = reached_goal;
        if (client.call(set_bool)) {
            ROS_DEBUG_STREAM(set_bool.response.message);
        } else {
            ROS_ERROR("Failed to call service reached_goal");
        }
    };
    if (way_points_idx < waypoints_.size() && is_close_local_goal(current_pose_, waypoints_[way_points_idx])) {
        way_points_idx++;
    }
    if (way_points_idx == waypoints_.size() && RANDOM_WAYPOINTS) {
        const auto &candidate = points_relation_[next_pos_idx_];
        int random_idx = dist_(mt_) % candidate.size();
        if (candidate[random_idx] == pre_pos_idx_) {
            random_idx++;
            random_idx %= candidate.size();
        }
        pre_pos_idx_ = next_pos_idx_;
        next_pos_idx_ = candidate[random_idx];
        waypoints_.push_back(points_[next_pos_idx_]);
    } else if (way_points_idx == waypoints_.size() && LOOP_WAYPOINTS) {
        way_points_idx = 0;
    } else if (way_points_idx >= waypoints_.size()) {  // >= for when clear_waypoints is called
        if (way_points_idx != waypoints_.size()) way_points_idx = waypoints_.size();
        ROS_INFO_THROTTLE(15.0, "Robot reached last waypoint.");
        if (!reached_goal_) {
            call_reached_goal(reached_goal_client_, true);
            reached_goal_ = true;
        }
        return;
    } else if (reached_goal_) {
        call_reached_goal(reached_goal_client_, false);
        reached_goal_ = false;
    }
    geometry_msgs::PoseStamped local_goal = waypoints_[way_points_idx];
    if (way_points_idx > 0) calc_local_goal(waypoints_[way_points_idx - 1], waypoints_[way_points_idx], &local_goal);
    local_goal.header.stamp = ros::Time::now();
    local_goal_pub_.publish(local_goal);
}

void WaypointsManager::calc_local_goal(const geometry_msgs::PoseStamped &pre_waypoint,
                                       const geometry_msgs::PoseStamped &next_waypoint,
                                       geometry_msgs::PoseStamped *local_goal) {
    double x_p = pre_waypoint.pose.position.x;
    double y_p = pre_waypoint.pose.position.y;
    double x_n = next_waypoint.pose.position.x;
    double y_n = next_waypoint.pose.position.y;
    double a1, b1;
    get_line(x_p, y_p, x_n, y_n, &a1, &b1);
    double x_c = current_pose_.pose.pose.position.x;
    double y_c = current_pose_.pose.pose.position.y;
    double a2, b2;
    get_vertical_line(a1, x_c, y_c, &a2, &b2);
    double x_t, y_t;
    get_intersection(a1, b1, a2, b2, &x_t, &y_t);
    double x_s, y_s;
    get_advance_point(x_p, y_p, x_n, y_n, x_t, y_t, ADVANCE_LENGTH, &x_s, &y_s);
    if ((x_s < x_p || x_n < x_s) && (x_s < x_n || x_p < x_s)) {
        x_s = x_n;
        y_s = y_n;
    }
    double a3, b3;
    get_vertical_line(a1, x_s, y_s, &a3, &b3);
    double x_l, y_l;
    get_vertical_advance_point(x_p, y_p, x_n, y_n, x_s, y_s, RIGHT_DISTANCE, &x_l, &y_l);
    local_goal->pose.orientation.w = 1.0;
    local_goal->pose.orientation.x = 0.0;
    local_goal->pose.orientation.y = 0.0;
    local_goal->pose.orientation.z = 0.0;
    local_goal->pose.position.x = x_l;
    local_goal->pose.position.y = y_l;
    local_goal->pose.position.z = 0;
}

// y = ax + b
void WaypointsManager::get_line(double x1, double y1, double x2, double y2, double *a, double *b) {
    double denominator = x2 - x1;
    if (std::abs(denominator) < 0.0001) denominator = 0.0001;
    *a = (y2 - y1) / denominator;
    *b = *a * (-x1) + y1;
}

void WaypointsManager::get_vertical_line(double a_src, double x, double y, double *a, double *b) {
    *a = -1.0 / a_src;
    *b = y - *a * x;
}

void WaypointsManager::get_intersection(double a1, double b1, double a2, double b2, double *x, double *y) {
    // a1*x + b1 = a2*x + b2
    // (a1 - a2) x = b2 - b1
    // x = (b2 - b1) / (a1 - a2)
    double denominator = a1 - a2;
    if (std::abs(denominator) < 0.0001) denominator = 0.0001;
    *x = (b2 - b1) / denominator;
    *y = a1 * *x + b1;
}

void WaypointsManager::get_advance_point(double x1, double y1, double x2, double y2, double x_src, double y_src,
                                         double len, double *x, double *y) {
    double sq = std::sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
    double dx = (x2 - x1) / sq;
    double dy = (y2 - y1) / sq;
    *x = x_src + dx * len;
    *y = y_src + dy * len;
}

void WaypointsManager::get_vertical_advance_point(double x1, double y1, double x2, double y2, double x_src,
                                                  double y_src, double len, double *x, double *y) {
    double sq = std::sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
    double dx = (y2 - y1) / sq;
    double dy = -(x2 - x1) / sq;
    *x = x_src + dx * len;
    *y = y_src + dy * len;
}

void WaypointsManager::pose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &amcl_pose) {
    current_pose_ = *amcl_pose;
}

bool WaypointsManager::is_close_local_goal(const geometry_msgs::PoseWithCovarianceStamped &amcl_pose,
                                           const geometry_msgs::PoseStamped &local_goal) {
    double diff_x = amcl_pose.pose.pose.position.x - local_goal.pose.position.x;
    double diff_y = amcl_pose.pose.pose.position.y - local_goal.pose.position.y;
    if (std::sqrt(diff_x * diff_x + diff_y * diff_y) <= GOAL_THRESHOLD) return true;
    return false;
}

void WaypointsManager::process() {
    timer = nh_.createTimer(ros::Duration(1.0 / HZ), &WaypointsManager::timer_callback, this);
    ros::spin();
}
