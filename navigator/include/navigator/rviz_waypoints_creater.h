#ifndef MULTI_ROBOTS_RVIZ_WAYPOINTS_CREATER_H_
#define MULTI_ROBOTS_RVIZ_WAYPOINTS_CREATER_H_

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

#include <fstream>
#include <iomanip>

class RvizWaypointsCreater {
 public:
    RvizWaypointsCreater();
    void pose_callback(const geometry_msgs::PoseStampedConstPtr &pose);
    void timer_callback(const ros::TimerEvent &event);
    void read_waypoints(std::vector<geometry_msgs::PoseStamped> *waypoints);
    void process();

 private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Timer timer;
    ros::Subscriber pose_sub_;
    ros::Publisher pose_pub_;

    std::string filename;
    std::vector<geometry_msgs::PoseStamped> points_;
};

#endif  // MULTI_ROBOTS_RVIZ_WAYPOINTS_CREATER_H_
