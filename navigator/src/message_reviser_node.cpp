#include "navigator/message_reviser.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "message_reviser");
    MessageReviser message_reviser;
    message_reviser.process();
    return 0;
}
