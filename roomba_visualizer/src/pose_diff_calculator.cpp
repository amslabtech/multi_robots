#include <color_detector_msgs/TargetPosition.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <ros/ros.h>
#include <signal.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <fstream>

class PoseDiffCalculator {
 public:
    PoseDiffCalculator();
    void process();
    void callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &forward_roomba_pose,
                  const geometry_msgs::PoseWithCovarianceStampedConstPtr &back_roomba_pose,
                  const color_detector_msgs::TargetPositionConstPtr &target);
    void calc_target_pose_on_world(const color_detector_msgs::TargetPositionConstPtr &target,
                                   const geometry_msgs::TransformStamped &transform,
                                   geometry_msgs::PoseStamped *target_pose);

 private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    std::string BACK_ROOMBA;
    std::string opath;
    std::ofstream ofs;
    message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> forward_pose_sub_;
    message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> back_pose_sub_;
    message_filters::Subscriber<color_detector_msgs::TargetPosition> target_sub_;
    typedef message_filters::sync_policies::ApproximateTime<
        geometry_msgs::PoseWithCovarianceStamped, geometry_msgs::PoseWithCovarianceStamped,
        color_detector_msgs::TargetPosition>
        sync_policy;
    message_filters::Synchronizer<sync_policy> synchronizer_;
    ros::Publisher pose_pub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

PoseDiffCalculator::PoseDiffCalculator()
    : private_nh_("~"),
      forward_pose_sub_(nh_, "forward/amcl_pose", 10),
      back_pose_sub_(nh_, "back/amcl_pose", 10),
      target_sub_(nh_, "target/position", 10),
      synchronizer_(sync_policy(10), forward_pose_sub_, back_pose_sub_, target_sub_),
      tf_listener_(tf_buffer_) {
    private_nh_.param("BACK_ROOMBA", BACK_ROOMBA, std::string("back_roomba"));
    opath = "/home/amsl/bagfiles/roomba/six_roomba_20210513/csvs/" + BACK_ROOMBA + "_pose_diff.csv";
    ofs.open(opath);
    ofs << ",forward,amcl,back,amcl,forward,realsense,realsense,data,,,,forward,diff," << std::endl;
    ofs << "time,x,y,x,y,x,y,x,y,z,count,sqrt(x^2+z^2),x,y,sqrt(x^2+y^2)" << std::endl;
    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("tf_pose", 1);
    synchronizer_.registerCallback(boost::bind(&PoseDiffCalculator::callback, this, _1, _2, _3));
}

void PoseDiffCalculator::callback(
    const geometry_msgs::PoseWithCovarianceStampedConstPtr &forward_roomba_pose,
    const geometry_msgs::PoseWithCovarianceStampedConstPtr &back_roomba_pose,
    const color_detector_msgs::TargetPositionConstPtr &target) {
    geometry_msgs::TransformStamped transform_stamped;
    try {
        transform_stamped = tf_buffer_.lookupTransform("map", BACK_ROOMBA + "/laser", ros::Time(0));
    } catch (tf2::TransformException &ex) {
        ROS_WARN_STREAM(ex.what());
        ros::Duration(1.0).sleep();
        return;
    }
    geometry_msgs::PoseStamped target_pose;
    calc_target_pose_on_world(target, transform_stamped, &target_pose);
    pose_pub_.publish(target_pose);
    ROS_INFO_STREAM("published transformed pose");
    double sqrtxz = std::sqrt(target->x * target->x + target->z * target->z);
    double diff_x = target_pose.pose.position.x - forward_roomba_pose->pose.pose.position.x;
    double diff_y = target_pose.pose.position.y - forward_roomba_pose->pose.pose.position.y;
    double sqrtxy = std::sqrt(diff_x * diff_x + diff_y * diff_y);
    static ros::Time start = ros::Time::now();
    ofs << (ros::Time::now() - start).toSec() << ',' << forward_roomba_pose->pose.pose.position.x
        << ',' << forward_roomba_pose->pose.pose.position.y << ','
        << back_roomba_pose->pose.pose.position.x << ',' << back_roomba_pose->pose.pose.position.y
        << ',' << target_pose.pose.position.x << ',' << target_pose.pose.position.y << ','
        << target->x << ',' << target->y << ',' << target->z << ',' << target->cluster_num << ','
        << sqrtxz << ',' << diff_x << ',' << diff_y << ',' << sqrtxy << std::endl;
    return;
}

void PoseDiffCalculator::calc_target_pose_on_world(
    const color_detector_msgs::TargetPositionConstPtr &target,
    const geometry_msgs::TransformStamped &transform, geometry_msgs::PoseStamped *output_pose) {
    geometry_msgs::PoseStamped target_pose;
    target_pose.header = target->header;
    target_pose.header.frame_id = BACK_ROOMBA + "/laser";
    target_pose.pose.position.x = target->z;
    target_pose.pose.position.y = -target->x;
    target_pose.pose.position.z = target->y;
    target_pose.pose.orientation.w = 1;
    target_pose.pose.orientation.x = 0;
    target_pose.pose.orientation.y = 0;
    target_pose.pose.orientation.z = 0;

    tf2::doTransform(target_pose, *output_pose, transform);
    return;
}

void PoseDiffCalculator::process() {
    ros::spin();
    return;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "pose_diff_calculator");
    PoseDiffCalculator pose_diff_calculator;
    pose_diff_calculator.process();
    return 0;
}
