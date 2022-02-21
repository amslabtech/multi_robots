#include "image_color_detector/image_color_detector.h"

ImageColorDetector::ImageColorDetector() : ImageColorDetector(ros::NodeHandle(), ros::NodeHandle("~")) {}

ImageColorDetector::ImageColorDetector(ros::NodeHandle nh, ros::NodeHandle private_nh)
    : nh_(nh), private_nh_(private_nh) {
    image_sub_ = nh_.subscribe("equirectangular/image_raw", 1, &ImageColorDetector::image_callback, this);
    target_angle_list_pub_ = nh_.advertise<color_detector_msgs::TargetAngleList>("target/angle", 1);

    private_nh_.param("ONLY_PUBLISH_MASK_IMAGE", only_publish_mask_image_, false);
    private_nh_.param("PUBLISH_TARGET_IMAGE", publish_target_image_, false);
    private_nh_.param("ROOMBA", ROOMBA, std::string("roomba0"));
    private_nh_.param("MIN_CLUSTER_SIZE", MIN_CLUSTER_SIZE, 20);
    private_nh_.param("MAX_CLUSTER_SIZE", MAX_CLUSTER_SIZE, 5000);
    color_detector_params_hsv::init(colors_, param_hsvs_);
    set_hsv_params();
    set_image_pubs();
}

void ImageColorDetector::set_hsv_params() {
    for (size_t i = 0; i < colors_.size(); i++) {
        std::string uppercase_latter;
        uppercase_latter.resize(colors_[i].size());
        std::transform(colors_[i].begin(), colors_[i].end(), uppercase_latter.begin(), toupper);
        private_nh_.param(ROOMBA + "_LOWER_" + uppercase_latter + "_H", param_hsvs_[i].lower.h, 0);
        private_nh_.param(ROOMBA + "_LOWER_" + uppercase_latter + "_S", param_hsvs_[i].lower.s, 0);
        private_nh_.param(ROOMBA + "_LOWER_" + uppercase_latter + "_V", param_hsvs_[i].lower.v, 0);
        private_nh_.param(ROOMBA + "_UPPER_" + uppercase_latter + "_H", param_hsvs_[i].upper.h, 0);
        private_nh_.param(ROOMBA + "_UPPER_" + uppercase_latter + "_S", param_hsvs_[i].upper.s, 0);
        private_nh_.param(ROOMBA + "_UPPER_" + uppercase_latter + "_V", param_hsvs_[i].upper.v, 0);
    }
}

void ImageColorDetector::set_image_pubs() {
    target_image_pubs_.resize(colors_.size());
    masked_image_pubs_.resize(colors_.size());
    for (size_t i = 0; i < colors_.size(); i++) {
        target_image_pubs_[i] = private_nh_.advertise<sensor_msgs::Image>("target/" + colors_[i] + "/image_raw", 1);
        masked_image_pubs_[i] = private_nh_.advertise<sensor_msgs::Image>("masked/" + colors_[i] + "/image_raw", 1);
    }
}

void ImageColorDetector::image_callback(const sensor_msgs::ImageConstPtr &received_image) {
    auto start_time = ros::Time::now();
    cv::Mat bgr_image;
    to_cv_image(*received_image, bgr_image);
    cv::Mat hsv_image;
    cv::cvtColor(bgr_image, hsv_image, CV_BGR2HSV);

    color_detector_msgs::TargetAngleList targets;
    targets.header = received_image->header;
    targets.my_number = ROOMBA.back() - '0';
    for (size_t i = 0; i < colors_.size(); i++) {
        cv::Mat binarized_hsv_image;
        filter_hsv(hsv_image, param_hsvs_[i], binarized_hsv_image);
        if (only_publish_mask_image_) {
            sensor_msgs::ImagePtr image_msg;
            cv::Mat masked_image;
            cv::bitwise_and(bgr_image, bgr_image, masked_image, binarized_hsv_image);
            image_msg = cv_bridge::CvImage(received_image->header, "bgr8", masked_image).toImageMsg();
            masked_image_pubs_[i].publish(image_msg);
            continue;
        }
        cv::Mat target_image;
        std::vector<std::pair<int, int>> target_pixels;
        const int mag = cv::countNonZero(binarized_hsv_image) / MAX_CLUSTER_SIZE + 1;
        ROS_DEBUG_STREAM("masked_image_pixel_num [" << colors_[i] << "] : " << cv::countNonZero(binarized_hsv_image));
        detect_target(binarized_hsv_image, mag, target_image, target_pixels);
        color_detector_msgs::TargetAngle target_msg;
        create_target_msg(colors_[i], bgr_image.cols, mag, target_pixels, target_msg);
        targets.data.push_back(target_msg);
        if (publish_target_image_) {
            sensor_msgs::ImagePtr image_msg;
            create_target_image(received_image->header, bgr_image, target_pixels, image_msg);
            target_image_pubs_[i].publish(image_msg);
            ROS_INFO_STREAM("target pixel num [" << colors_[i] << "] : " << target_pixels.size() * mag
                                                 << " (mag=" << mag << ")");
        }
    }
    target_angle_list_pub_.publish(targets);
    ROS_INFO_STREAM("[image_color_detector] elasped time : " << (ros::Time::now() - start_time).toSec() << "[sec]");
}

void ImageColorDetector::to_cv_image(const sensor_msgs::Image &ros_image, cv::Mat &output_image) {
    cv_bridge::CvImageConstPtr cv_image_ptr;
    try {
        cv_image_ptr = cv_bridge::toCvCopy(ros_image, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &ex) {
        ROS_ERROR_STREAM("cv_bridge exception: " << ex.what());
        return;
    }

    cv::Mat cv_image(cv_image_ptr->image.rows, cv_image_ptr->image.cols, cv_image_ptr->image.type());
    cv_image = cv_image_ptr->image;
    output_image = std::move(cv_image);
    return;
}

void ImageColorDetector::filter_hsv(const cv::Mat &hsv_image, const ThresholdHSV &thres_hsv, cv::Mat &output_image) {
    if (thres_hsv.lower.h <= thres_hsv.upper.h) {
        std::vector<int> upper = {thres_hsv.upper.h, thres_hsv.upper.s, thres_hsv.upper.v};
        std::vector<int> lower = {thres_hsv.lower.h, thres_hsv.lower.s, thres_hsv.lower.v};
        cv::inRange(hsv_image, lower, upper, output_image);
    } else {
        cv::Mat image1, image2;
        std::vector<int> upper = {180, thres_hsv.upper.s, thres_hsv.upper.v};
        std::vector<int> lower = {thres_hsv.lower.h, thres_hsv.lower.s, thres_hsv.lower.v};
        cv::inRange(hsv_image, lower, upper, image1);
        upper = {thres_hsv.upper.h, thres_hsv.upper.s, thres_hsv.upper.v};
        lower = {0, thres_hsv.lower.s, thres_hsv.lower.v};
        cv::inRange(hsv_image, lower, upper, image2);
        cv::bitwise_or(image1, image2, output_image);
    }
    return;
}

void ImageColorDetector::detect_target(const cv::Mat &binarized_image, int mag, cv::Mat &output_image,
                                       std::vector<std::pair<int, int>> &output_pixels) {
    const int H = binarized_image.rows;
    const int W = binarized_image.cols;

    std::vector<std::vector<bool>> reached(H, std::vector<bool>(W, false));
    std::vector<std::pair<int, int>> target_pixels;
    for (size_t h = 0; h < H; h += mag) {
        for (size_t w = 0; w < W; w += mag) {
            if (reached[h][w]) continue;
            std::vector<std::pair<int, int>> pixels;
            form_cluster(h, w, mag, binarized_image, reached, pixels);
            if (pixels.size() > target_pixels.size()) {
                target_pixels = std::move(pixels);
            }
        }
    }

    output_pixels = std::move(target_pixels);
    return;
}

void ImageColorDetector::form_cluster(int start_x, int start_y, int mag, const cv::Mat &mat,
                                      std::vector<std::vector<bool>> &reached,
                                      std::vector<std::pair<int, int>> &output_pixels) {
    const std::vector<std::pair<int, int>> move_direction = {{0, mag}, {mag, 0}, {0, -mag}, {-mag, 0}};

    std::queue<std::pair<int, int>> que;
    que.emplace(start_x, start_y);
    reached[start_x][start_y] = true;
    std::vector<std::pair<int, int>> pixels;
    while (!que.empty()) {
        int h, w;
        std::tie(h, w) = que.front();
        que.pop();

        for (const auto &m : move_direction) {
            int next_h = h + m.first;
            int next_w = w + m.second;
            if (next_h < 0 || mat.rows <= next_h || next_w < 0 || mat.cols <= next_w || reached[next_h][next_w])
                continue;
            if (mat.at<uchar>(h, w) == 0) continue;
            que.emplace(next_h, next_w);
            reached[next_h][next_w] = true;
            pixels.emplace_back(next_h, next_w);
        }
    }

    output_pixels = std::move(pixels);
    return;
}

void ImageColorDetector::create_target_msg(std::string color, int width, int mag,
                                           const std::vector<std::pair<int, int>> &pixels,
                                           color_detector_msgs::TargetAngle &output_msg) {
    color_detector_msgs::TargetAngle target_msg;
    target_msg.color = color;
    target_msg.cluster_num = pixels.size() * mag;
    // 0 <= x < H, 0 <= y < W
    int sum_x = 0;
    int sum_y = 0;
    for (const auto &pixel : pixels) {
        sum_x += pixel.first;
        sum_y += pixel.second;
    }
    double ave_x = 1. * sum_x / pixels.size();
    double ave_y = 1. * sum_y / pixels.size();
    target_msg.angle = calc_angle(ave_y, width);
    target_msg.radian = target_msg.angle / 180. * M_PI;

    output_msg = std::move(target_msg);
    return;
}

void ImageColorDetector::create_target_image(const std_msgs::Header &header, const cv::Mat &bgr_image,
                                             const std::vector<std::pair<int, int>> &pixels,
                                             sensor_msgs::ImagePtr &output_msg) {
    cv::Mat target_image(bgr_image.rows, bgr_image.cols, bgr_image.type());
    for (int i = 0; i < target_image.rows; i++) {
        for (int j = 0; j < target_image.cols; j++) {
            target_image.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 0);
        }
    }
    for (const auto &pixel : pixels) {
        target_image.at<cv::Vec3b>(pixel.first, pixel.second) = bgr_image.at<cv::Vec3b>(pixel.first, pixel.second);
    }

    output_msg = cv_bridge::CvImage(header, "bgr8", target_image).toImageMsg();
    return;
}

double ImageColorDetector::calc_angle(double target_position, int width) {
    return -180. + 360. * target_position / width;
}

void ImageColorDetector::process() { ros::spin(); }
