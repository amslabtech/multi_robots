#include "image_editor/image_editor.h"

int main(int argc,char** argv)
{
    ros::init(argc,argv,"image_editor");
    ImageEditor image_editor;
    image_editor.cv_process();
    return 0;
}
