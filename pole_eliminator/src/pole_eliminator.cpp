#include "pole_eliminator/pole_eliminator.h"

PoleEliminator::PoleEliminator() : private_nh_("~") {
    private_nh_.param("HZ", HZ, 40);
    private_nh_.param("ROBOT_RADIUS", ROBOT_RADIUS, 0.30);
    private_nh_.param("MARGIN", MARGIN, 15);
    private_nh_.param("LASER_FRAME", LASER_FRAME, std::string(""));
    private_nh_.param("LINEAR_INTERPOLATE_THRESHOLD", LINEAR_INTERPOLATE_THRESHOLD, M_PI / 6.0);

    raw_laser_scan_sub_ = nh_.subscribe("scan", 10, &PoleEliminator::laser_scan_callback, this);
    corrected_laser_scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>("corrected_scan", 1);
}

void PoleEliminator::laser_scan_callback(const sensor_msgs::LaserScanConstPtr &raw_laser) {
    const size_t LASER_SIZE = raw_laser->ranges.size();
    ROS_ASSERT(LASER_SIZE <= MAX_LASER_SIZE);

    static std::array<bool, MAX_LASER_SIZE> is_in_radius;
    check_laser_is_in_radius(raw_laser, &is_in_radius);
    extend_is_in_radius(MARGIN, LASER_SIZE, &is_in_radius);

    //  is_in_radius[i] == true (first <= i < second)
    std::vector<std::pair<size_t, size_t>> pole_ranges;
    check_ranges_is_in_radius(LASER_SIZE, is_in_radius, &pole_ranges);
    ROS_DEBUG("Auto detector poles:");
    for (const auto &range : pole_ranges) {
        ROS_DEBUG_STREAM("[" << range.first << ", " << range.second << ")");
    }

    std::vector<std::function<double(double)>> linear_interpolation_funcs;
    register_calculation_function(raw_laser, pole_ranges, &linear_interpolation_funcs);
    ROS_ASSERT(pole_ranges.size() == linear_interpolation_funcs.size());

    sensor_msgs::LaserScan corrected_laser = *raw_laser;
    if (!LASER_FRAME.empty()) corrected_laser.header.frame_id = LASER_FRAME;
    for (size_t range_idx = 0; range_idx < pole_ranges.size(); range_idx++) {
        for (size_t i = pole_ranges[range_idx].first; i < pole_ranges[range_idx].second; i++) {
            double angle = ranges_index_to_angle(i, corrected_laser.angle_min, corrected_laser.angle_increment);
            corrected_laser.ranges[i] = linear_interpolation_funcs[range_idx](angle);
            if (corrected_laser.ranges[i] < ROBOT_RADIUS) corrected_laser.ranges[i] = 30.0;
        }
    }
    corrected_laser_scan_pub_.publish(corrected_laser);
}

void PoleEliminator::check_laser_is_in_radius(const sensor_msgs::LaserScanConstPtr &laser,
                                              std::array<bool, MAX_LASER_SIZE> *is_in_radius_ptr) {
    for (size_t i = 0; i < laser->ranges.size(); i++) {
        if (laser->ranges[i] < ROBOT_RADIUS)
            is_in_radius_ptr->at(i) = true;
        else
            is_in_radius_ptr->at(i) = false;
    }
}

void PoleEliminator::extend_is_in_radius(int margin, const size_t LASER_SIZE,
                                         std::array<bool, MAX_LASER_SIZE> *is_in_radius_ptr) {
    std::vector<int> extend_back, extend_forward;
    for (size_t i = 0; i < LASER_SIZE; i++) {
        if (i > 0 && !(is_in_radius_ptr->at(i - 1)) && is_in_radius_ptr->at(i)) extend_back.push_back(i);
        if (i < LASER_SIZE - 1 && is_in_radius_ptr->at(i) && !(is_in_radius_ptr->at(i + 1)))
            extend_forward.push_back(i);
    }
    for (auto idx : extend_back) {
        for (size_t i = std::max(0, idx - margin); i < idx; i++) {
            is_in_radius_ptr->at(i) = true;
        }
    }
    for (auto idx : extend_forward) {
        for (size_t i = idx + 1; i <= std::min(idx + margin, static_cast<int>(LASER_SIZE) - 1); i++) {
            is_in_radius_ptr->at(i) = true;
        }
    }
}

void PoleEliminator::check_ranges_is_in_radius(const size_t LASER_SIZE,
                                               const std::array<bool, MAX_LASER_SIZE> &is_in_radius,
                                               std::vector<std::pair<size_t, size_t>> *pole_ranges_ptr) {
    bool is_started = false;
    size_t st_idx, end_idx;
    for (size_t i = 0; i < LASER_SIZE; i++) {
        if (!is_started && is_in_radius[i]) {
            st_idx = i;
            is_started = true;
        }
        if (is_started && !(is_in_radius[i])) {
            end_idx = i;
            is_started = false;
            pole_ranges_ptr->emplace_back(st_idx, end_idx);
        }
    }
    if (is_started) {
        pole_ranges_ptr->emplace_back(st_idx, LASER_SIZE);
    }
}

bool PoleEliminator::is_linear_interpolated(const sensor_msgs::LaserScanConstPtr &laser, size_t min_idx,
                                            size_t max_idx) {
    double pre_b, b, next_b, c;
    if (min_idx - MARGIN < 0 || laser->ranges.size() <= max_idx + MARGIN) return false;
    calc_constant_bc(laser, min_idx - MARGIN, min_idx, &pre_b, &c);
    double pre_theta = std::atan(pre_b);
    calc_constant_bc(laser, min_idx, max_idx, &b, &c);
    double theta = std::atan(b);
    calc_constant_bc(laser, max_idx, max_idx + MARGIN, &next_b, &c);
    double next_theta = std::atan(next_b);
    if (std::abs(pre_theta - theta) < LINEAR_INTERPOLATE_THRESHOLD &&
        std::abs(theta - next_theta) < LINEAR_INTERPOLATE_THRESHOLD) {
        return true;
    }
    return false;
}

void PoleEliminator::register_calculation_function(
    const sensor_msgs::LaserScanConstPtr &laser, const std::vector<std::pair<size_t, size_t>> &pole_ranges,
    std::vector<std::function<double(double)>> *linear_interpolation_funcs_ptr) {
    for (const auto &range : pole_ranges) {
        ROS_ASSERT(!(range.first == 0 && range.second == laser->ranges.size()));
        if (range.first == 0) {
            linear_interpolation_funcs_ptr->push_back([&](double angle) { return laser->ranges[range.second]; });
        } else if (range.second == laser->ranges.size()) {
            linear_interpolation_funcs_ptr->push_back([&](double angle) { return laser->ranges[range.first]; });
        } else if (is_linear_interpolated(laser, range.first, range.second)) {
            double b, c;
            calc_constant_bc(laser, range.first, range.second - 1, &b, &c);
            linear_interpolation_funcs_ptr->push_back(
                std::bind(&PoleEliminator::calc_range, this, b, c, std::placeholders::_1));
        } else {
            linear_interpolation_funcs_ptr->push_back([&](double angle) { return laser->range_max; });
        }
    }
}

void PoleEliminator::calc_constant_bc(const sensor_msgs::LaserScanConstPtr &laser, size_t min_idx, size_t max_idx,
                                      double *b_ptr, double *c_ptr) {
    double min_theta = ranges_index_to_angle(min_idx, laser->angle_min, laser->angle_increment);
    double max_theta = ranges_index_to_angle(max_idx, laser->angle_min, laser->angle_increment);

    double min_point_x = laser->ranges[min_idx] * std::cos(min_theta);
    double min_point_y = laser->ranges[min_idx] * std::sin(min_theta);
    double max_point_x = laser->ranges[max_idx] * std::cos(max_theta);
    double max_point_y = laser->ranges[max_idx] * std::sin(max_theta);

    if (min_point_x == max_point_x) ROS_FATAL_STREAM("PoleEliminator::calc_constant_bc contains a division by zero");
    // y = bx + c
    *b_ptr = (min_point_y - max_point_y) / (min_point_x - max_point_x);
    *c_ptr = (max_point_x * min_point_y - min_point_x * max_point_y) / (max_point_x - min_point_x);
}

double PoleEliminator::ranges_index_to_angle(size_t index, double angle_min, double angle_increase) {
    return angle_min + index * angle_increase;
}

double PoleEliminator::calc_range(double b, double c, double angle) {
    static auto is_equal = [](double val1, double val2) -> bool {
        constexpr double eps = 0.0001;
        if (std::abs(val1 - val2) < eps) return true;
        return false;
    };
    double cross_x, cross_y;
    if (is_equal(0.0, angle) || is_equal(M_PI, angle) || is_equal(-M_PI, angle)) {
        cross_x = -c / b;
        cross_y = 0.0;
    } else if (is_equal(M_PI / 2.0, angle) || is_equal(-M_PI / 2.0, angle)) {
        cross_x = 0.0;
        cross_y = c;
    } else {                         // cross_y = b * cross_x + c
        double a = std::tan(angle);  // a = cross_y / cross_x
        if (a == b) ROS_FATAL_STREAM("PoleEliminator::calc_range contains a division by zero");
        cross_x = c / (a - b);
        cross_y = a * c / (a - b);
    }
    return std::sqrt(cross_x * cross_x + cross_y * cross_y);
}

void PoleEliminator::process() {
    ros::Rate loop_rate(HZ);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
}
