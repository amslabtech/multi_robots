#include <ros/ros.h>

#include "color_detector_srvs/ColorEnable.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "teleop_color_enable");
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<color_detector_srvs::ColorEnable>("color_enable");

    const std::vector<std::string> colors = {"green", "red", "blue", "yellow", "white"};
    while (true) {
        std::cout << "input color (green, red, blue, yellow, white)" << std::endl;
        std::string input_color;
        std::cin >> input_color;
        if (input_color == "q") break;
        bool exist = false;
        for (auto color : colors) {
            if (color == input_color) {
                exist = true;
                break;
            }
        }
        if (!exist) {
            std::cerr << "You input " << input_color << ". Only use 'green', 'red', 'blue', 'yellow' or 'white'." << std::endl;
            continue;
        }

        std::cout << "input enable (true, false)" << std::endl;
        std::string input_str_enable;
        std::cin >> input_str_enable;
        if (input_str_enable == "q") break;
        bool enable;
        if (input_str_enable == "true") {
            enable = true;
        } else if (input_str_enable == "false") {
            enable = false;
        } else {
            std::cerr << "You input " << input_str_enable << ". Only use 'true' or 'false'." << std::endl;
            continue;
        }

        color_detector_srvs::ColorEnable srv;
        srv.request.color = input_color;
        srv.request.enable = enable;
        if (client.call(srv)) {
            ROS_INFO_STREAM("Success to call service color_enable");
        } else {
            ROS_ERROR_STREAM("Failed to call service color_enable");
        }
    }

    return 0;
}