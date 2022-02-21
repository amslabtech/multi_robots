#include "image_color_detector/image_color_detector.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "image_color_detector");
    ImageColorDetector image_color_detector;
    image_color_detector.process();
    return 0;
}
