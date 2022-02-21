#ifndef MULTI_ROBOTS_WAYPOINTS_MANAGER_H_
#define MULTI_ROBOTS_WAYPOINTS_MANAGER_H_

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>

#include <fstream>
#include <iomanip>
#include <random>

class WaypointsManager {
 public:
    WaypointsManager();
    void process();
    void read_waypoints(std::vector<geometry_msgs::PoseStamped> *waypoints);
    void read_points_relation();
    void random_waypoint_to_yaml(int size);
    void timer_callback(const ros::TimerEvent &event);
    void calc_local_goal(const geometry_msgs::PoseStamped &pre_waypoint,
                         const geometry_msgs::PoseStamped &next_waypoint, geometry_msgs::PoseStamped *local_goal);
    void get_line(double x1, double y1, double x2, double y2, double *a, double *b);
    void get_vertical_line(double a_src, double x, double y, double *a, double *b);
    void get_intersection(double a1, double b1, double a2, double b2, double *x, double *y);
    void get_advance_point(double x1, double y1, double x2, double y2, double x_src, double y_src, double len,
                           double *x, double *y);
    void get_vertical_advance_point(double x1, double y1, double x2, double y2, double x_src, double y_src, double len,
                                    double *x, double *y);
    void rviz_local_goal_callback(const geometry_msgs::PoseStampedConstPtr &pose);
    bool clear_waypoints_service(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);
    void pose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &amcl_pose);
    bool is_close_local_goal(const geometry_msgs::PoseWithCovarianceStamped &amcl_pose,
                             const geometry_msgs::PoseStamped &local_goal);

 private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Timer timer;
    ros::Subscriber pose_sub_;
    ros::Subscriber local_goal_sub_;
    ros::Publisher local_goal_pub_;
    ros::ServiceClient reached_goal_client_;
    ros::ServiceServer clear_waypoints_server_;
    int HZ;
    bool LOOP_WAYPOINTS;
    bool RANDOM_WAYPOINTS;
    int EXPORT_WAYPOINTS_SIZE;
    double GOAL_THRESHOLD;
    double ADVANCE_LENGTH;
    double RIGHT_DISTANCE;
    geometry_msgs::PoseWithCovarianceStamped current_pose_;
    std::vector<geometry_msgs::PoseStamped> waypoints_;
    bool reached_goal_;
    // for RAMDOM_WAYPOINTS
    std::vector<geometry_msgs::PoseStamped> points_;
    std::vector<std::vector<int>> points_relation_;
    int pre_pos_idx_;
    int next_pos_idx_;
    std::mt19937 mt_;
    std::uniform_int_distribution<int> dist_;
};

#endif  // MULTI_ROBOTS_WAYPOINTS_MANAGER_H_
