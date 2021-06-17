#ifndef MULTI_ROBOTS_POLE_ELIMINATOR_H_
#define MULTI_ROBOTS_POLE_ELIMINATOR_H_

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

class PoleEliminator {
 private:
    int HZ;
    double ROBOT_RADIUS;
    int MARGIN;
    std::string LASER_FRAME;
    static constexpr size_t MAX_LASER_SIZE = 10000;
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber raw_laser_scan_sub_;
    ros::Publisher corrected_laser_scan_pub_;

 public:
    PoleEliminator();
    void process();
    void laser_scan_callback(const sensor_msgs::LaserScanConstPtr &raw_laser);
    void check_laser_is_in_radius(const sensor_msgs::LaserScanConstPtr &laser,
                                  std::array<bool, MAX_LASER_SIZE> *is_in_radius_ptr);
    void extend_is_in_radius(int margin, const size_t LASER_SIZE,
                             std::array<bool, MAX_LASER_SIZE> *is_in_radius_ptr);
    void check_ranges_is_in_radius(const size_t LASER_SIZE,
                                   const std::array<bool, MAX_LASER_SIZE> &is_in_radius,
                                   std::vector<std::pair<size_t, size_t>> *pole_ranges_ptr);
    void register_calculation_function(
        const sensor_msgs::LaserScanConstPtr &laser,
        const std::vector<std::pair<size_t, size_t>> &pole_ranges,
        std::vector<std::function<double(double)>> *linear_interpolation_funcs_ptr);
    void calc_constant_bc(const sensor_msgs::LaserScanConstPtr &laser, size_t min_idx,
                          size_t max_idx, double *b_ptr, double *c_ptr);
    double ranges_index_to_angle(size_t index, double angle_min, double angle_increase);
    double calc_range(double b, double c, double angle);
};

#endif  // MULTI_ROBOTS_POLE_ELIMINATOR_H_
