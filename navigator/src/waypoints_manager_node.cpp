#include "navigator/waypoints_manager.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "waypoints_manager");
    WaypointsManager waypoints_manager;
    waypoints_manager.process();
    return 0;
}
