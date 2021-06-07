#include "image_editor.h"

ImageEditor::ImageEditor():private_nh("")
{
    private_nh.param("th",th,{0.9});
    image_sub = nh.subscribe("/equirectangular/image_raw",10,&ImageEditor::cv_image_callback,this);
    image_pub = nh.advertise<sensor_msgs::Image>("equirectangular/cv_image_edited",1);
}

void ImageEditor::cv_image_callback(const sensor_msgs::ImageConstPtr& cv_sub_image)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(cv_sub_image, sensor_msgs::image_encodings::BGR8);
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_brige exception: %s",e.what());
        return;
    }

    cv::Mat cv_image(cv_ptr->image.rows, cv_ptr->image.cols, cv_ptr->image.type());
    cv_image = cv_ptr->image;

    cv::Mat cv_image_2(cv_ptr->image.rows, cv_ptr->image.cols, cv_ptr->image.type());
    cv_image_2 = cv_ptr->image;

    cv::Ptr<cv::xphoto::GrayworldWB> wb = cv::xphoto::createGrayworldWB();
    wb -> setSaturationThreshold(th);
    wb -> balanceWhite(cv_image,cv_image_2);

    //toRosmsg
    cv_pub_image = cv_bridge::CvImage(std_msgs::Header(),"bgr8",cv_image_2).toImageMsg();
    cv_pub_image->header = cv_sub_image->header;
    image_pub.publish(cv_pub_image);

    return;
}

void ImageEditor::cv_process()
{
    ros::spin();
    return;
}

int main(int argc,char** argv)
{
    ros::init(argc,argv,"image_editor");
    ImageEditor image_editor;
    image_editor.cv_process();
    return 0;
}

