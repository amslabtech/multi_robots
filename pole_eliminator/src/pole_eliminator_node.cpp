#include "pole_eliminator/pole_eliminator.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "pole_eliminator");
    PoleEliminator pole_eliminator;
    pole_eliminator.process();
    return 0;
}
