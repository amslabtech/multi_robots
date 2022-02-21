#include "navigator/target_follower.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "target_follower");
    TargetFollower target_follower;
    target_follower.process();
    return 0;
}
