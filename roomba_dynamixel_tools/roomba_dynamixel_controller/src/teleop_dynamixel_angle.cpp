#include <ros/ros.h>

#include <iostream>
#include <cmath>

#include "dynamixel_angle_msgs/DynamixelAngle.h"

bool is_number(const std::string& str)
{
    std::string sub_str = str;
    if(sub_str.substr(0,1) == "-") sub_str.erase(0,1);
    for(char const &c : sub_str){
        if(std::isdigit(c) == 0) return false;
        else return true;
    }
}

int main(int argc,char **argv)
{
    ros::init(argc,argv,"teleop_angle");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<dynamixel_angle_msgs::DynamixelAngle>("/angle",1);

    while(1){
        std::cout << "Input a 'q' if you want to exit" << std::endl;
        std::cout << "Input an angle [rad] (-M_PI ~ M_PI): ";
        std::string input;
        std::cin >> input;
        if(is_number(input)){
            dynamixel_angle_msgs::DynamixelAngle angle;
            angle.header.stamp = ros::Time::now();
            angle.angle = std::stof(input);
            pub.publish(angle);
            std::cout << "Success to publish angle: " << angle.angle << "[rad]" << std::endl << std::endl;
        }else{
            if(input == "q") break;
            std::cerr << "The input value is invalid" << std::endl << std::endl;
        }
    }

    return 0;
}
