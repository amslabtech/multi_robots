#ifndef POLE_ELIMINATOR_H
#define POLE_ELIMINATOR_H

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "math.h"

class Pole_Eliminator
{
 public:
     Pole_Eliminator();
     void process();

 private:
    //method
    void laser_scan_callback(const sensor_msgs::LaserScan::ConstPtr&);
    //parameter
    int hz;
    int width;
    int height;
    double resolution;
    int number_of_laser = 1080;
    int row;
    int column;
    int radius_limit;
    int pole_min_idx[4];
    int pole_max_idx[4];
    bool is_edge;

    //member
    ros::NodeHandle nh;
    ros::NodeHandle private_nh;
    ros::Publisher pub_laser_scan;
    ros::Subscriber sub_laser_scan;
    sensor_msgs::LaserScan scan_data;
    sensor_msgs::LaserScan corrected_scan_data;

};
#endif//POLE_ELIMINATOR_H
