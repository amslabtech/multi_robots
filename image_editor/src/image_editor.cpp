#include "image_editor.h"

ImageEditor::ImageEditor():private_nh("")
{
    sub_image = nh.subscribe("/equirectangular/image_raw",1,&ImageEditor::image_callback,this);
    pub_image = nh.advertise<sensor_msgs::Image>("/equirectangular/image_edited",1);
}

ImageConverter::ImageConverter():private_nh("")
{
    private_nh.param("th",th,{0.9});
    image_sub = nh.subscribe("/equirectangular/image_raw",10,&ImageConverter::cv_image_callback,this);
    image_pub = nh.advertise<sensor_msgs::Image>("equirectangular/cv_image_edited",1);

    // cv::namedWindow(OPENCV_WINDOW);
}
// ~ImageConverter()
// {
//     cv::destroyWindow(OPENCV_WINDOW);
// }
//
void ImageConverter::cv_image_callback(const sensor_msgs::ImageConstPtr& cv_sub_image)
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


    // std::cout<<"(WhiteBalance)"<<std::endl;

    cv::Ptr<cv::xphoto::GrayworldWB> wb = cv::xphoto::createGrayworldWB();
    // th = wb -> getSaturationThreshold();
    // std::cout<<"th = "<<th<<std::endl;
    // if(th != 0.9f)
    // {
    //     std::cout<<std::fixed<<std::setprecision(7) <<wb -> getSaturationThreshold()<<std::endl;
    // }
    wb -> setSaturationThreshold(th);
    wb -> balanceWhite(cv_image,cv_image_2);

    //toRosmsg
    cv_pub_image = cv_bridge::CvImage(std_msgs::Header(),"bgr8",cv_image_2).toImageMsg();
    cv_pub_image->header = cv_sub_image->header;
    image_pub.publish(cv_pub_image);

    return;
}

void ImageConverter::cv_process()
{
    ros::spin();
    return;
}


void ImageEditor::image_callback(const sensor_msgs::Image::ConstPtr &msg)
{
    raw_image = *msg;

    row = raw_image.height;
    col = raw_image.width;
    step = raw_image.step;

    edited_image.height = row;
    edited_image.width = col;
    edited_image.step = step;

    raw_data = std::vector<int>(row*step);
    edited_data = std::vector<int>(row*step);

    edited_image.header = msg->header;
    edited_image.encoding = msg->encoding;
    edited_image.is_bigendian = msg->is_bigendian;
    edited_image.data.resize(row*step);
}

void ImageEditor::image_edit()
{

    std::cout<<raw_image.step<<std::endl;
    std::cout<<col<<std::endl;
    std::cout<<row<<std::endl;
    std::cout<<"======"<<std::endl;


    ROS_WARN_STREAM("raw data size : " << raw_data.size());
    ROS_WARN_STREAM("edit data size : " << edited_data.size());
    ROS_WARN_STREAM("raw image data size : " << raw_image.data.size());
    ROS_WARN_STREAM("raw image data size : " << edited_image.data.size());
    for(int i=0;i<row*step;i++)
    {
        raw_data[i] = raw_image.data[i];
        //std::cout<<raw_data[i]<<std::endl;
        edited_data[i] = raw_data[i];
        //std::cout<<raw_data[i]<<","<<edited_data[i]<<std::endl;
        //edited_image.data[i] = edited_data[i];
    }

    for(int j=0;j<row*step;j++)
    {
        edited_image.data[j] = edited_data[j];
    }

    std::cout<<"------"<<std::endl;
    pub_image.publish(edited_image);
}

void ImageEditor::process()
{
    ros::Rate loop_rate(10);
    int  i = 0;
    while(ros::ok())
    {

        if(i>5){

        image_edit();
        }
        i++;

        ros::spinOnce();
        loop_rate.sleep();
    }
}


int main(int argc,char** argv)
{
    ros::init(argc,argv,"image_editor");
    //ImageEditor image_editor;
    ImageConverter image_converter;

    //std::cout<<"image_edit"<<std::endl;
    std::cout<<"image_convert"<<std::endl;
    //image_editor.process();
    image_converter.cv_process();
    return 0;
}

