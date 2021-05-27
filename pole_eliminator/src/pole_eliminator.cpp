#include "pole_eliminator/pole_eliminator.h"

Pole_Eliminator::Pole_Eliminator():private_nh("~")
{
    //parameter
    private_nh.param("hz",hz,{1});
    private_nh.param("width",width,{5});
    private_nh.param("height",height,{5});
    private_nh.param("resolution",resolution,{0.05});
    private_nh.param("radius_limit",radius_limit,{49});

    //subscriber
    sub_laser_scan = nh.subscribe("scan",10,&Pole_Eliminator::laser_scan_callback,this);
    //publisher
    pub_laser_scan = nh.advertise<sensor_msgs::LaserScan>("corrected_scan", 10);
}


void Pole_Eliminator::laser_scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    // std::cout<<"recieved scan_data"<<std::endl;
    scan_data = *msg;

    static bool first = true;
    //if (!first) return;
    ROS_INFO_STREAM("recieved data size : " << msg->ranges.size());
    bool f = false;
    double dist = 0.20;
    double a = -1.0;
    double b = 0.0;
    double c = 0.0;
    double point_min[2] = {0.0, 0.0};
    double point_max[2] = {0.0, 0.0};
    double cross_point[2] = {0.0, 0.0};
    int counter = 0;

    for(size_t i=0;i<4;i++){
       pole_min_idx[i] = 0;
       pole_max_idx[i] = 0;
    }

    for (size_t i = 0; i < msg->ranges.size(); i++) {
        if (scan_data.ranges[i] < dist && !f) {
            f = true;
            ROS_INFO_STREAM("from " << counter << ", " << i);
            if(i==0)
               pole_min_idx[counter] = i;
            else
               pole_min_idx[counter] = i-10;
            if(i > 1000){
               pole_min_idx[counter] = i-10;
               pole_max_idx[counter] = 1081;
            }
        }
        if (scan_data.ranges[i] >= dist && f) {
            f = false;
            ROS_INFO_STREAM("     " << counter << ", " << i << " end");
            pole_max_idx[counter] = i+10;
        }

        if((counter==0 || counter == 3) && pole_max_idx[counter] - pole_min_idx[counter] > 20)
            counter++;
        if((counter==1 || counter == 2) && pole_max_idx[counter] - pole_min_idx[counter] > 30)
            counter++;
    }

    ROS_INFO_STREAM("Auto detector");
    for(size_t i=0;i<4;i++){
        ROS_INFO_STREAM("from " << pole_min_idx[i]);
        ROS_INFO_STREAM("     " << pole_max_idx[i] << " end");
    }
    first = false;

    counter = 0;
    point_min[0] = scan_data.ranges[pole_min_idx[counter]]*sin((-45.0+pole_min_idx[counter]*0.25)*M_PI/180.0);
    point_min[1] = scan_data.ranges[pole_min_idx[counter]]*cos((-45.0+pole_min_idx[counter]*0.25)*M_PI/180.0);
    point_max[0] = scan_data.ranges[pole_max_idx[counter]]*sin((-45.0+pole_max_idx[counter]*0.25)*M_PI/180.0);
    point_max[1] = scan_data.ranges[pole_max_idx[counter]]*cos((-45.0+pole_max_idx[counter]*0.25)*M_PI/180.0);
    ROS_INFO_STREAM("point_min " << point_min[0] << ", " << point_min[1] << ", " << sqrt(pow(point_min[0], 2.0) + pow(point_min[0], 2.0)));
    ROS_INFO_STREAM("point_max " << point_max[0] << ", " << point_max[1] << ", " << sqrt(pow(point_max[0], 2.0) + pow(point_max[0], 2.0)));
    for(size_t i=0;i<msg->ranges.size();i++){
        if(pole_min_idx[counter] <= i && i <= pole_max_idx[counter]){
                 b=(point_min[1]-point_max[1])/(point_min[0]-point_max[0]);
                 c=(point_max[0]*point_min[1]-point_min[0]*point_max[1])/(point_max[0]-point_min[0]);    
            if(-45.0+i*0.25 == 0.0 || -45.0+i*0.25 == 180.0){
                 cross_point[0] = -c/b;
                 cross_point[1] = 0.0;
            }else if(-45.0+i*0.25 == 90.0 || -45.0+i*0.25 == 270.0){            
                 cross_point[0] = 0.0;
                 cross_point[1] = c;
            }else{
                 a = tan((-45.0+i*0.25)*M_PI/180.0);
                 cross_point[0] = c/(a-b);
                 cross_point[1] = a*c/(a-b);
            }
            
            //ROS_INFO_STREAM("cross_point " << cross_point[0] << ", " << cross_point[1]);
            //if(pole_min_idx[counter] > dist){
            scan_data.ranges[i] = sqrt(pow(cross_point[0],2.0)+pow(cross_point[1],2.0));
            scan_data.intensities[i] = scan_data.intensities[pole_min_idx[counter]];
            
            //ROS_INFO_STREAM("Log a b c " << a << ", " << b << ", " << c);
            ROS_INFO_STREAM("scan_data.ranges " << scan_data.ranges[i]);
            //}else{
               //scan_data.ranges[i] = scan_data.ranges[pole_max_idx[counter]];
               //scan_data.intensities[i] = scan_data.intensities[pole_max_idx[counter]];            
            //}
            
        }else if(pole_max_idx[counter] < i){
            counter++;
            point_min[0] = scan_data.ranges[pole_min_idx[counter]]*sin((-45.0+pole_min_idx[counter]*0.25)*M_PI/180.0);
            point_min[1] = scan_data.ranges[pole_min_idx[counter]]*cos((-45.0+pole_min_idx[counter]*0.25)*M_PI/180.0);
            point_max[0] = scan_data.ranges[pole_max_idx[counter]]*sin((-45.0+pole_max_idx[counter]*0.25)*M_PI/180.0);
            point_max[1] = scan_data.ranges[pole_max_idx[counter]]*cos((-45.0+pole_max_idx[counter]*0.25)*M_PI/180.0);
            ROS_INFO_STREAM("point_min " << point_min[0] << ", " << point_min[1] << ", " << sqrt(pow(point_min[0], 2.0) + pow(point_min[0], 2.0)));
            ROS_INFO_STREAM("point_max " << point_max[0] << ", " << point_max[1] << ", " << sqrt(pow(point_max[0], 2.0) + pow(point_max[0], 2.0)));
        }

    }

    ROS_INFO_STREAM("Received");
    corrected_scan_data = scan_data;
    pub_laser_scan.publish(corrected_scan_data);
}




void Pole_Eliminator::process()
{
    ros::Rate loop_rate(hz);
    while(ros::ok())
    {
        // add_frame_local_map();
        ros::spin();
        loop_rate.sleep();
    }
}

int main (int argc, char **argv)
{
    ros::init(argc,argv,"pole_eliminator");
    Pole_Eliminator pole_eliminator;
    pole_eliminator.process();
    return 0;
}
