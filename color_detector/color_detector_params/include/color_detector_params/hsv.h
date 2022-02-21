#ifndef COLOR_DETECTOR_PARAMS_HSV_H_
#define COLOR_DETECTOR_PARAMS_HSV_H_

#include <ros/node_handle.h>

#include <algorithm>
#include <string>
#include <vector>

#include "color_detector_params/HsvConfig.h"

namespace color_detector_params_hsv {
struct HSV {
    int h, s, v;
};

struct ThresholdHSV {
    HSV lower, upper;
};

inline void init(std::vector<std::string> &colors, std::vector<ThresholdHSV> &config_hsvs) {
    colors = {"apricot", "green", "yellow", "blue", "orange", "purple", "red", "brown", "navy", "olive"};
    config_hsvs.resize(colors.size());
}

inline void init(std::vector<std::string> &colors) {
    std::vector<ThresholdHSV> _;
    init(colors, _);
}

inline void update_use_colors(const std::vector<std::string> &colors, const color_detector_params::HsvConfig &config,
                              std::vector<bool> &use_colors) {
    for (size_t i = 0; i < colors.size(); i++) {
        if (colors[i] == "red") use_colors[i] = config.red_enable;
        if (colors[i] == "blue") use_colors[i] = config.blue_enable;
        if (colors[i] == "green") use_colors[i] = config.green_enable;
        if (colors[i] == "yellow") use_colors[i] = config.yellow_enable;
        if (colors[i] == "orange") use_colors[i] = config.orange_enable;
        if (colors[i] == "purple") use_colors[i] = config.purple_enable;
        if (colors[i] == "apricot") use_colors[i] = config.apricot_enable;
        if (colors[i] == "navy") use_colors[i] = config.navy_enable;
        if (colors[i] == "olive") use_colors[i] = config.olive_enable;
        if (colors[i] == "brown") use_colors[i] = config.brown_enable;
    }
}
inline void set_hsv(const std::vector<std::string> &colors, const color_detector_params::HsvConfig &config, int i,
                    std::vector<ThresholdHSV> &config_hsvs) {
    if (colors[i] == "red") {
        config_hsvs[i].lower.h = config.LOWER_RED_H;
        config_hsvs[i].lower.s = config.LOWER_RED_S;
        config_hsvs[i].lower.v = config.LOWER_RED_V;
        config_hsvs[i].upper.h = config.UPPER_RED_H;
        config_hsvs[i].upper.s = config.UPPER_RED_S;
        config_hsvs[i].upper.v = config.UPPER_RED_V;
    } else if (colors[i] == "green") {
        config_hsvs[i].lower.h = config.LOWER_GREEN_H;
        config_hsvs[i].lower.s = config.LOWER_GREEN_S;
        config_hsvs[i].lower.v = config.LOWER_GREEN_V;
        config_hsvs[i].upper.h = config.UPPER_GREEN_H;
        config_hsvs[i].upper.s = config.UPPER_GREEN_S;
        config_hsvs[i].upper.v = config.UPPER_GREEN_V;
    } else if (colors[i] == "blue") {
        config_hsvs[i].lower.h = config.LOWER_BLUE_H;
        config_hsvs[i].lower.s = config.LOWER_BLUE_S;
        config_hsvs[i].lower.v = config.LOWER_BLUE_V;
        config_hsvs[i].upper.h = config.UPPER_BLUE_H;
        config_hsvs[i].upper.s = config.UPPER_BLUE_S;
        config_hsvs[i].upper.v = config.UPPER_BLUE_V;
    } else if (colors[i] == "yellow") {
        config_hsvs[i].lower.h = config.LOWER_YELLOW_H;
        config_hsvs[i].lower.s = config.LOWER_YELLOW_S;
        config_hsvs[i].lower.v = config.LOWER_YELLOW_V;
        config_hsvs[i].upper.h = config.UPPER_YELLOW_H;
        config_hsvs[i].upper.s = config.UPPER_YELLOW_S;
        config_hsvs[i].upper.v = config.UPPER_YELLOW_V;
    } else if (colors[i] == "orange") {
        config_hsvs[i].lower.h = config.LOWER_ORANGE_H;
        config_hsvs[i].lower.s = config.LOWER_ORANGE_S;
        config_hsvs[i].lower.v = config.LOWER_ORANGE_V;
        config_hsvs[i].upper.h = config.UPPER_ORANGE_H;
        config_hsvs[i].upper.s = config.UPPER_ORANGE_S;
        config_hsvs[i].upper.v = config.UPPER_ORANGE_V;
    } else if (colors[i] == "purple") {
        config_hsvs[i].lower.h = config.LOWER_PURPLE_H;
        config_hsvs[i].lower.s = config.LOWER_PURPLE_S;
        config_hsvs[i].lower.v = config.LOWER_PURPLE_V;
        config_hsvs[i].upper.h = config.UPPER_PURPLE_H;
        config_hsvs[i].upper.s = config.UPPER_PURPLE_S;
        config_hsvs[i].upper.v = config.UPPER_PURPLE_V;
    } else if (colors[i] == "apricot") {
        config_hsvs[i].lower.h = config.LOWER_APRICOT_H;
        config_hsvs[i].lower.s = config.LOWER_APRICOT_S;
        config_hsvs[i].lower.v = config.LOWER_APRICOT_V;
        config_hsvs[i].upper.h = config.UPPER_APRICOT_H;
        config_hsvs[i].upper.s = config.UPPER_APRICOT_S;
        config_hsvs[i].upper.v = config.UPPER_APRICOT_V;
    } else if (colors[i] == "navy") {
        config_hsvs[i].lower.h = config.LOWER_NAVY_H;
        config_hsvs[i].lower.s = config.LOWER_NAVY_S;
        config_hsvs[i].lower.v = config.LOWER_NAVY_V;
        config_hsvs[i].upper.h = config.UPPER_NAVY_H;
        config_hsvs[i].upper.s = config.UPPER_NAVY_S;
        config_hsvs[i].upper.v = config.UPPER_NAVY_V;
    } else if (colors[i] == "olive") {
        config_hsvs[i].lower.h = config.LOWER_OLIVE_H;
        config_hsvs[i].lower.s = config.LOWER_OLIVE_S;
        config_hsvs[i].lower.v = config.LOWER_OLIVE_V;
        config_hsvs[i].upper.h = config.UPPER_OLIVE_H;
        config_hsvs[i].upper.s = config.UPPER_OLIVE_S;
        config_hsvs[i].upper.v = config.UPPER_OLIVE_V;
    } else if (colors[i] == "brown") {
        config_hsvs[i].lower.h = config.LOWER_BROWN_H;
        config_hsvs[i].lower.s = config.LOWER_BROWN_S;
        config_hsvs[i].lower.v = config.LOWER_BROWN_V;
        config_hsvs[i].upper.h = config.UPPER_BROWN_H;
        config_hsvs[i].upper.s = config.UPPER_BROWN_S;
        config_hsvs[i].upper.v = config.UPPER_BROWN_V;
    }
}

inline void set_roomba1_hsv(const std::vector<std::string> &colors, const color_detector_params::HsvConfig &config,
                            int i, std::vector<ThresholdHSV> &config_hsvs) {
    if (colors[i] == "red") {
        config_hsvs[i].lower.h = config.ROOMBA1_LOWER_RED_H;
        config_hsvs[i].lower.s = config.ROOMBA1_LOWER_RED_S;
        config_hsvs[i].lower.v = config.ROOMBA1_LOWER_RED_V;
        config_hsvs[i].upper.h = config.ROOMBA1_UPPER_RED_H;
        config_hsvs[i].upper.s = config.ROOMBA1_UPPER_RED_S;
        config_hsvs[i].upper.v = config.ROOMBA1_UPPER_RED_V;
    } else if (colors[i] == "green") {
        config_hsvs[i].lower.h = config.ROOMBA1_LOWER_GREEN_H;
        config_hsvs[i].lower.s = config.ROOMBA1_LOWER_GREEN_S;
        config_hsvs[i].lower.v = config.ROOMBA1_LOWER_GREEN_V;
        config_hsvs[i].upper.h = config.ROOMBA1_UPPER_GREEN_H;
        config_hsvs[i].upper.s = config.ROOMBA1_UPPER_GREEN_S;
        config_hsvs[i].upper.v = config.ROOMBA1_UPPER_GREEN_V;
    } else if (colors[i] == "blue") {
        config_hsvs[i].lower.h = config.ROOMBA1_LOWER_BLUE_H;
        config_hsvs[i].lower.s = config.ROOMBA1_LOWER_BLUE_S;
        config_hsvs[i].lower.v = config.ROOMBA1_LOWER_BLUE_V;
        config_hsvs[i].upper.h = config.ROOMBA1_UPPER_BLUE_H;
        config_hsvs[i].upper.s = config.ROOMBA1_UPPER_BLUE_S;
        config_hsvs[i].upper.v = config.ROOMBA1_UPPER_BLUE_V;
    } else if (colors[i] == "yellow") {
        config_hsvs[i].lower.h = config.ROOMBA1_LOWER_YELLOW_H;
        config_hsvs[i].lower.s = config.ROOMBA1_LOWER_YELLOW_S;
        config_hsvs[i].lower.v = config.ROOMBA1_LOWER_YELLOW_V;
        config_hsvs[i].upper.h = config.ROOMBA1_UPPER_YELLOW_H;
        config_hsvs[i].upper.s = config.ROOMBA1_UPPER_YELLOW_S;
        config_hsvs[i].upper.v = config.ROOMBA1_UPPER_YELLOW_V;
    } else if (colors[i] == "orange") {
        config_hsvs[i].lower.h = config.ROOMBA1_LOWER_ORANGE_H;
        config_hsvs[i].lower.s = config.ROOMBA1_LOWER_ORANGE_S;
        config_hsvs[i].lower.v = config.ROOMBA1_LOWER_ORANGE_V;
        config_hsvs[i].upper.h = config.ROOMBA1_UPPER_ORANGE_H;
        config_hsvs[i].upper.s = config.ROOMBA1_UPPER_ORANGE_S;
        config_hsvs[i].upper.v = config.ROOMBA1_UPPER_ORANGE_V;
    } else if (colors[i] == "purple") {
        config_hsvs[i].lower.h = config.ROOMBA1_LOWER_PURPLE_H;
        config_hsvs[i].lower.s = config.ROOMBA1_LOWER_PURPLE_S;
        config_hsvs[i].lower.v = config.ROOMBA1_LOWER_PURPLE_V;
        config_hsvs[i].upper.h = config.ROOMBA1_UPPER_PURPLE_H;
        config_hsvs[i].upper.s = config.ROOMBA1_UPPER_PURPLE_S;
        config_hsvs[i].upper.v = config.ROOMBA1_UPPER_PURPLE_V;
    } else if (colors[i] == "apricot") {
        config_hsvs[i].lower.h = config.ROOMBA1_LOWER_APRICOT_H;
        config_hsvs[i].lower.s = config.ROOMBA1_LOWER_APRICOT_S;
        config_hsvs[i].lower.v = config.ROOMBA1_LOWER_APRICOT_V;
        config_hsvs[i].upper.h = config.ROOMBA1_UPPER_APRICOT_H;
        config_hsvs[i].upper.s = config.ROOMBA1_UPPER_APRICOT_S;
        config_hsvs[i].upper.v = config.ROOMBA1_UPPER_APRICOT_V;
    } else if (colors[i] == "navy") {
        config_hsvs[i].lower.h = config.ROOMBA1_LOWER_NAVY_H;
        config_hsvs[i].lower.s = config.ROOMBA1_LOWER_NAVY_S;
        config_hsvs[i].lower.v = config.ROOMBA1_LOWER_NAVY_V;
        config_hsvs[i].upper.h = config.ROOMBA1_UPPER_NAVY_H;
        config_hsvs[i].upper.s = config.ROOMBA1_UPPER_NAVY_S;
        config_hsvs[i].upper.v = config.ROOMBA1_UPPER_NAVY_V;
    } else if (colors[i] == "olive") {
        config_hsvs[i].lower.h = config.ROOMBA1_LOWER_OLIVE_H;
        config_hsvs[i].lower.s = config.ROOMBA1_LOWER_OLIVE_S;
        config_hsvs[i].lower.v = config.ROOMBA1_LOWER_OLIVE_V;
        config_hsvs[i].upper.h = config.ROOMBA1_UPPER_OLIVE_H;
        config_hsvs[i].upper.s = config.ROOMBA1_UPPER_OLIVE_S;
        config_hsvs[i].upper.v = config.ROOMBA1_UPPER_OLIVE_V;
    } else if (colors[i] == "brown") {
        config_hsvs[i].lower.h = config.ROOMBA1_LOWER_BROWN_H;
        config_hsvs[i].lower.s = config.ROOMBA1_LOWER_BROWN_S;
        config_hsvs[i].lower.v = config.ROOMBA1_LOWER_BROWN_V;
        config_hsvs[i].upper.h = config.ROOMBA1_UPPER_BROWN_H;
        config_hsvs[i].upper.s = config.ROOMBA1_UPPER_BROWN_S;
        config_hsvs[i].upper.v = config.ROOMBA1_UPPER_BROWN_V;
    }
}

inline void set_roomba2_hsv(const std::vector<std::string> &colors, const color_detector_params::HsvConfig &config,
                            int i, std::vector<ThresholdHSV> &config_hsvs) {
    if (colors[i] == "red") {
        config_hsvs[i].lower.h = config.ROOMBA2_LOWER_RED_H;
        config_hsvs[i].lower.s = config.ROOMBA2_LOWER_RED_S;
        config_hsvs[i].lower.v = config.ROOMBA2_LOWER_RED_V;
        config_hsvs[i].upper.h = config.ROOMBA2_UPPER_RED_H;
        config_hsvs[i].upper.s = config.ROOMBA2_UPPER_RED_S;
        config_hsvs[i].upper.v = config.ROOMBA2_UPPER_RED_V;
    } else if (colors[i] == "green") {
        config_hsvs[i].lower.h = config.ROOMBA2_LOWER_GREEN_H;
        config_hsvs[i].lower.s = config.ROOMBA2_LOWER_GREEN_S;
        config_hsvs[i].lower.v = config.ROOMBA2_LOWER_GREEN_V;
        config_hsvs[i].upper.h = config.ROOMBA2_UPPER_GREEN_H;
        config_hsvs[i].upper.s = config.ROOMBA2_UPPER_GREEN_S;
        config_hsvs[i].upper.v = config.ROOMBA2_UPPER_GREEN_V;
    } else if (colors[i] == "blue") {
        config_hsvs[i].lower.h = config.ROOMBA2_LOWER_BLUE_H;
        config_hsvs[i].lower.s = config.ROOMBA2_LOWER_BLUE_S;
        config_hsvs[i].lower.v = config.ROOMBA2_LOWER_BLUE_V;
        config_hsvs[i].upper.h = config.ROOMBA2_UPPER_BLUE_H;
        config_hsvs[i].upper.s = config.ROOMBA2_UPPER_BLUE_S;
        config_hsvs[i].upper.v = config.ROOMBA2_UPPER_BLUE_V;
    } else if (colors[i] == "yellow") {
        config_hsvs[i].lower.h = config.ROOMBA2_LOWER_YELLOW_H;
        config_hsvs[i].lower.s = config.ROOMBA2_LOWER_YELLOW_S;
        config_hsvs[i].lower.v = config.ROOMBA2_LOWER_YELLOW_V;
        config_hsvs[i].upper.h = config.ROOMBA2_UPPER_YELLOW_H;
        config_hsvs[i].upper.s = config.ROOMBA2_UPPER_YELLOW_S;
        config_hsvs[i].upper.v = config.ROOMBA2_UPPER_YELLOW_V;
    } else if (colors[i] == "orange") {
        config_hsvs[i].lower.h = config.ROOMBA2_LOWER_ORANGE_H;
        config_hsvs[i].lower.s = config.ROOMBA2_LOWER_ORANGE_S;
        config_hsvs[i].lower.v = config.ROOMBA2_LOWER_ORANGE_V;
        config_hsvs[i].upper.h = config.ROOMBA2_UPPER_ORANGE_H;
        config_hsvs[i].upper.s = config.ROOMBA2_UPPER_ORANGE_S;
        config_hsvs[i].upper.v = config.ROOMBA2_UPPER_ORANGE_V;
    } else if (colors[i] == "purple") {
        config_hsvs[i].lower.h = config.ROOMBA2_LOWER_PURPLE_H;
        config_hsvs[i].lower.s = config.ROOMBA2_LOWER_PURPLE_S;
        config_hsvs[i].lower.v = config.ROOMBA2_LOWER_PURPLE_V;
        config_hsvs[i].upper.h = config.ROOMBA2_UPPER_PURPLE_H;
        config_hsvs[i].upper.s = config.ROOMBA2_UPPER_PURPLE_S;
        config_hsvs[i].upper.v = config.ROOMBA2_UPPER_PURPLE_V;
    } else if (colors[i] == "apricot") {
        config_hsvs[i].lower.h = config.ROOMBA2_LOWER_APRICOT_H;
        config_hsvs[i].lower.s = config.ROOMBA2_LOWER_APRICOT_S;
        config_hsvs[i].lower.v = config.ROOMBA2_LOWER_APRICOT_V;
        config_hsvs[i].upper.h = config.ROOMBA2_UPPER_APRICOT_H;
        config_hsvs[i].upper.s = config.ROOMBA2_UPPER_APRICOT_S;
        config_hsvs[i].upper.v = config.ROOMBA2_UPPER_APRICOT_V;
    } else if (colors[i] == "navy") {
        config_hsvs[i].lower.h = config.ROOMBA2_LOWER_NAVY_H;
        config_hsvs[i].lower.s = config.ROOMBA2_LOWER_NAVY_S;
        config_hsvs[i].lower.v = config.ROOMBA2_LOWER_NAVY_V;
        config_hsvs[i].upper.h = config.ROOMBA2_UPPER_NAVY_H;
        config_hsvs[i].upper.s = config.ROOMBA2_UPPER_NAVY_S;
        config_hsvs[i].upper.v = config.ROOMBA2_UPPER_NAVY_V;
    } else if (colors[i] == "olive") {
        config_hsvs[i].lower.h = config.ROOMBA2_LOWER_OLIVE_H;
        config_hsvs[i].lower.s = config.ROOMBA2_LOWER_OLIVE_S;
        config_hsvs[i].lower.v = config.ROOMBA2_LOWER_OLIVE_V;
        config_hsvs[i].upper.h = config.ROOMBA2_UPPER_OLIVE_H;
        config_hsvs[i].upper.s = config.ROOMBA2_UPPER_OLIVE_S;
        config_hsvs[i].upper.v = config.ROOMBA2_UPPER_OLIVE_V;
    } else if (colors[i] == "brown") {
        config_hsvs[i].lower.h = config.ROOMBA2_LOWER_BROWN_H;
        config_hsvs[i].lower.s = config.ROOMBA2_LOWER_BROWN_S;
        config_hsvs[i].lower.v = config.ROOMBA2_LOWER_BROWN_V;
        config_hsvs[i].upper.h = config.ROOMBA2_UPPER_BROWN_H;
        config_hsvs[i].upper.s = config.ROOMBA2_UPPER_BROWN_S;
        config_hsvs[i].upper.v = config.ROOMBA2_UPPER_BROWN_V;
    }
}

inline void set_roomba3_hsv(const std::vector<std::string> &colors, const color_detector_params::HsvConfig &config,
                            int i, std::vector<ThresholdHSV> &config_hsvs) {
    if (colors[i] == "red") {
        config_hsvs[i].lower.h = config.ROOMBA3_LOWER_RED_H;
        config_hsvs[i].lower.s = config.ROOMBA3_LOWER_RED_S;
        config_hsvs[i].lower.v = config.ROOMBA3_LOWER_RED_V;
        config_hsvs[i].upper.h = config.ROOMBA3_UPPER_RED_H;
        config_hsvs[i].upper.s = config.ROOMBA3_UPPER_RED_S;
        config_hsvs[i].upper.v = config.ROOMBA3_UPPER_RED_V;
    } else if (colors[i] == "green") {
        config_hsvs[i].lower.h = config.ROOMBA3_LOWER_GREEN_H;
        config_hsvs[i].lower.s = config.ROOMBA3_LOWER_GREEN_S;
        config_hsvs[i].lower.v = config.ROOMBA3_LOWER_GREEN_V;
        config_hsvs[i].upper.h = config.ROOMBA3_UPPER_GREEN_H;
        config_hsvs[i].upper.s = config.ROOMBA3_UPPER_GREEN_S;
        config_hsvs[i].upper.v = config.ROOMBA3_UPPER_GREEN_V;
    } else if (colors[i] == "blue") {
        config_hsvs[i].lower.h = config.ROOMBA3_LOWER_BLUE_H;
        config_hsvs[i].lower.s = config.ROOMBA3_LOWER_BLUE_S;
        config_hsvs[i].lower.v = config.ROOMBA3_LOWER_BLUE_V;
        config_hsvs[i].upper.h = config.ROOMBA3_UPPER_BLUE_H;
        config_hsvs[i].upper.s = config.ROOMBA3_UPPER_BLUE_S;
        config_hsvs[i].upper.v = config.ROOMBA3_UPPER_BLUE_V;
    } else if (colors[i] == "yellow") {
        config_hsvs[i].lower.h = config.ROOMBA3_LOWER_YELLOW_H;
        config_hsvs[i].lower.s = config.ROOMBA3_LOWER_YELLOW_S;
        config_hsvs[i].lower.v = config.ROOMBA3_LOWER_YELLOW_V;
        config_hsvs[i].upper.h = config.ROOMBA3_UPPER_YELLOW_H;
        config_hsvs[i].upper.s = config.ROOMBA3_UPPER_YELLOW_S;
        config_hsvs[i].upper.v = config.ROOMBA3_UPPER_YELLOW_V;
    } else if (colors[i] == "orange") {
        config_hsvs[i].lower.h = config.ROOMBA3_LOWER_ORANGE_H;
        config_hsvs[i].lower.s = config.ROOMBA3_LOWER_ORANGE_S;
        config_hsvs[i].lower.v = config.ROOMBA3_LOWER_ORANGE_V;
        config_hsvs[i].upper.h = config.ROOMBA3_UPPER_ORANGE_H;
        config_hsvs[i].upper.s = config.ROOMBA3_UPPER_ORANGE_S;
        config_hsvs[i].upper.v = config.ROOMBA3_UPPER_ORANGE_V;
    } else if (colors[i] == "purple") {
        config_hsvs[i].lower.h = config.ROOMBA3_LOWER_PURPLE_H;
        config_hsvs[i].lower.s = config.ROOMBA3_LOWER_PURPLE_S;
        config_hsvs[i].lower.v = config.ROOMBA3_LOWER_PURPLE_V;
        config_hsvs[i].upper.h = config.ROOMBA3_UPPER_PURPLE_H;
        config_hsvs[i].upper.s = config.ROOMBA3_UPPER_PURPLE_S;
        config_hsvs[i].upper.v = config.ROOMBA3_UPPER_PURPLE_V;
    } else if (colors[i] == "apricot") {
        config_hsvs[i].lower.h = config.ROOMBA3_LOWER_APRICOT_H;
        config_hsvs[i].lower.s = config.ROOMBA3_LOWER_APRICOT_S;
        config_hsvs[i].lower.v = config.ROOMBA3_LOWER_APRICOT_V;
        config_hsvs[i].upper.h = config.ROOMBA3_UPPER_APRICOT_H;
        config_hsvs[i].upper.s = config.ROOMBA3_UPPER_APRICOT_S;
        config_hsvs[i].upper.v = config.ROOMBA3_UPPER_APRICOT_V;
    } else if (colors[i] == "navy") {
        config_hsvs[i].lower.h = config.ROOMBA3_LOWER_NAVY_H;
        config_hsvs[i].lower.s = config.ROOMBA3_LOWER_NAVY_S;
        config_hsvs[i].lower.v = config.ROOMBA3_LOWER_NAVY_V;
        config_hsvs[i].upper.h = config.ROOMBA3_UPPER_NAVY_H;
        config_hsvs[i].upper.s = config.ROOMBA3_UPPER_NAVY_S;
        config_hsvs[i].upper.v = config.ROOMBA3_UPPER_NAVY_V;
    } else if (colors[i] == "olive") {
        config_hsvs[i].lower.h = config.ROOMBA3_LOWER_OLIVE_H;
        config_hsvs[i].lower.s = config.ROOMBA3_LOWER_OLIVE_S;
        config_hsvs[i].lower.v = config.ROOMBA3_LOWER_OLIVE_V;
        config_hsvs[i].upper.h = config.ROOMBA3_UPPER_OLIVE_H;
        config_hsvs[i].upper.s = config.ROOMBA3_UPPER_OLIVE_S;
        config_hsvs[i].upper.v = config.ROOMBA3_UPPER_OLIVE_V;
    } else if (colors[i] == "brown") {
        config_hsvs[i].lower.h = config.ROOMBA3_LOWER_BROWN_H;
        config_hsvs[i].lower.s = config.ROOMBA3_LOWER_BROWN_S;
        config_hsvs[i].lower.v = config.ROOMBA3_LOWER_BROWN_V;
        config_hsvs[i].upper.h = config.ROOMBA3_UPPER_BROWN_H;
        config_hsvs[i].upper.s = config.ROOMBA3_UPPER_BROWN_S;
        config_hsvs[i].upper.v = config.ROOMBA3_UPPER_BROWN_V;
    }
}

inline void set_roomba4_hsv(const std::vector<std::string> &colors, const color_detector_params::HsvConfig &config,
                            int i, std::vector<ThresholdHSV> &config_hsvs) {
    if (colors[i] == "red") {
        config_hsvs[i].lower.h = config.ROOMBA4_LOWER_RED_H;
        config_hsvs[i].lower.s = config.ROOMBA4_LOWER_RED_S;
        config_hsvs[i].lower.v = config.ROOMBA4_LOWER_RED_V;
        config_hsvs[i].upper.h = config.ROOMBA4_UPPER_RED_H;
        config_hsvs[i].upper.s = config.ROOMBA4_UPPER_RED_S;
        config_hsvs[i].upper.v = config.ROOMBA4_UPPER_RED_V;
    } else if (colors[i] == "green") {
        config_hsvs[i].lower.h = config.ROOMBA4_LOWER_GREEN_H;
        config_hsvs[i].lower.s = config.ROOMBA4_LOWER_GREEN_S;
        config_hsvs[i].lower.v = config.ROOMBA4_LOWER_GREEN_V;
        config_hsvs[i].upper.h = config.ROOMBA4_UPPER_GREEN_H;
        config_hsvs[i].upper.s = config.ROOMBA4_UPPER_GREEN_S;
        config_hsvs[i].upper.v = config.ROOMBA4_UPPER_GREEN_V;
    } else if (colors[i] == "blue") {
        config_hsvs[i].lower.h = config.ROOMBA4_LOWER_BLUE_H;
        config_hsvs[i].lower.s = config.ROOMBA4_LOWER_BLUE_S;
        config_hsvs[i].lower.v = config.ROOMBA4_LOWER_BLUE_V;
        config_hsvs[i].upper.h = config.ROOMBA4_UPPER_BLUE_H;
        config_hsvs[i].upper.s = config.ROOMBA4_UPPER_BLUE_S;
        config_hsvs[i].upper.v = config.ROOMBA4_UPPER_BLUE_V;
    } else if (colors[i] == "yellow") {
        config_hsvs[i].lower.h = config.ROOMBA4_LOWER_YELLOW_H;
        config_hsvs[i].lower.s = config.ROOMBA4_LOWER_YELLOW_S;
        config_hsvs[i].lower.v = config.ROOMBA4_LOWER_YELLOW_V;
        config_hsvs[i].upper.h = config.ROOMBA4_UPPER_YELLOW_H;
        config_hsvs[i].upper.s = config.ROOMBA4_UPPER_YELLOW_S;
        config_hsvs[i].upper.v = config.ROOMBA4_UPPER_YELLOW_V;
    } else if (colors[i] == "orange") {
        config_hsvs[i].lower.h = config.ROOMBA4_LOWER_ORANGE_H;
        config_hsvs[i].lower.s = config.ROOMBA4_LOWER_ORANGE_S;
        config_hsvs[i].lower.v = config.ROOMBA4_LOWER_ORANGE_V;
        config_hsvs[i].upper.h = config.ROOMBA4_UPPER_ORANGE_H;
        config_hsvs[i].upper.s = config.ROOMBA4_UPPER_ORANGE_S;
        config_hsvs[i].upper.v = config.ROOMBA4_UPPER_ORANGE_V;
    } else if (colors[i] == "purple") {
        config_hsvs[i].lower.h = config.ROOMBA4_LOWER_PURPLE_H;
        config_hsvs[i].lower.s = config.ROOMBA4_LOWER_PURPLE_S;
        config_hsvs[i].lower.v = config.ROOMBA4_LOWER_PURPLE_V;
        config_hsvs[i].upper.h = config.ROOMBA4_UPPER_PURPLE_H;
        config_hsvs[i].upper.s = config.ROOMBA4_UPPER_PURPLE_S;
        config_hsvs[i].upper.v = config.ROOMBA4_UPPER_PURPLE_V;
    } else if (colors[i] == "apricot") {
        config_hsvs[i].lower.h = config.ROOMBA4_LOWER_APRICOT_H;
        config_hsvs[i].lower.s = config.ROOMBA4_LOWER_APRICOT_S;
        config_hsvs[i].lower.v = config.ROOMBA4_LOWER_APRICOT_V;
        config_hsvs[i].upper.h = config.ROOMBA4_UPPER_APRICOT_H;
        config_hsvs[i].upper.s = config.ROOMBA4_UPPER_APRICOT_S;
        config_hsvs[i].upper.v = config.ROOMBA4_UPPER_APRICOT_V;
    } else if (colors[i] == "navy") {
        config_hsvs[i].lower.h = config.ROOMBA4_LOWER_NAVY_H;
        config_hsvs[i].lower.s = config.ROOMBA4_LOWER_NAVY_S;
        config_hsvs[i].lower.v = config.ROOMBA4_LOWER_NAVY_V;
        config_hsvs[i].upper.h = config.ROOMBA4_UPPER_NAVY_H;
        config_hsvs[i].upper.s = config.ROOMBA4_UPPER_NAVY_S;
        config_hsvs[i].upper.v = config.ROOMBA4_UPPER_NAVY_V;
    } else if (colors[i] == "olive") {
        config_hsvs[i].lower.h = config.ROOMBA4_LOWER_OLIVE_H;
        config_hsvs[i].lower.s = config.ROOMBA4_LOWER_OLIVE_S;
        config_hsvs[i].lower.v = config.ROOMBA4_LOWER_OLIVE_V;
        config_hsvs[i].upper.h = config.ROOMBA4_UPPER_OLIVE_H;
        config_hsvs[i].upper.s = config.ROOMBA4_UPPER_OLIVE_S;
        config_hsvs[i].upper.v = config.ROOMBA4_UPPER_OLIVE_V;
    } else if (colors[i] == "brown") {
        config_hsvs[i].lower.h = config.ROOMBA4_LOWER_BROWN_H;
        config_hsvs[i].lower.s = config.ROOMBA4_LOWER_BROWN_S;
        config_hsvs[i].lower.v = config.ROOMBA4_LOWER_BROWN_V;
        config_hsvs[i].upper.h = config.ROOMBA4_UPPER_BROWN_H;
        config_hsvs[i].upper.s = config.ROOMBA4_UPPER_BROWN_S;
        config_hsvs[i].upper.v = config.ROOMBA4_UPPER_BROWN_V;
    }
}

inline void set_roomba5_hsv(const std::vector<std::string> &colors, const color_detector_params::HsvConfig &config,
                            int i, std::vector<ThresholdHSV> &config_hsvs) {
    if (colors[i] == "red") {
        config_hsvs[i].lower.h = config.ROOMBA5_LOWER_RED_H;
        config_hsvs[i].lower.s = config.ROOMBA5_LOWER_RED_S;
        config_hsvs[i].lower.v = config.ROOMBA5_LOWER_RED_V;
        config_hsvs[i].upper.h = config.ROOMBA5_UPPER_RED_H;
        config_hsvs[i].upper.s = config.ROOMBA5_UPPER_RED_S;
        config_hsvs[i].upper.v = config.ROOMBA5_UPPER_RED_V;
    } else if (colors[i] == "green") {
        config_hsvs[i].lower.h = config.ROOMBA5_LOWER_GREEN_H;
        config_hsvs[i].lower.s = config.ROOMBA5_LOWER_GREEN_S;
        config_hsvs[i].lower.v = config.ROOMBA5_LOWER_GREEN_V;
        config_hsvs[i].upper.h = config.ROOMBA5_UPPER_GREEN_H;
        config_hsvs[i].upper.s = config.ROOMBA5_UPPER_GREEN_S;
        config_hsvs[i].upper.v = config.ROOMBA5_UPPER_GREEN_V;
    } else if (colors[i] == "blue") {
        config_hsvs[i].lower.h = config.ROOMBA5_LOWER_BLUE_H;
        config_hsvs[i].lower.s = config.ROOMBA5_LOWER_BLUE_S;
        config_hsvs[i].lower.v = config.ROOMBA5_LOWER_BLUE_V;
        config_hsvs[i].upper.h = config.ROOMBA5_UPPER_BLUE_H;
        config_hsvs[i].upper.s = config.ROOMBA5_UPPER_BLUE_S;
        config_hsvs[i].upper.v = config.ROOMBA5_UPPER_BLUE_V;
    } else if (colors[i] == "yellow") {
        config_hsvs[i].lower.h = config.ROOMBA5_LOWER_YELLOW_H;
        config_hsvs[i].lower.s = config.ROOMBA5_LOWER_YELLOW_S;
        config_hsvs[i].lower.v = config.ROOMBA5_LOWER_YELLOW_V;
        config_hsvs[i].upper.h = config.ROOMBA5_UPPER_YELLOW_H;
        config_hsvs[i].upper.s = config.ROOMBA5_UPPER_YELLOW_S;
        config_hsvs[i].upper.v = config.ROOMBA5_UPPER_YELLOW_V;
    } else if (colors[i] == "orange") {
        config_hsvs[i].lower.h = config.ROOMBA5_LOWER_ORANGE_H;
        config_hsvs[i].lower.s = config.ROOMBA5_LOWER_ORANGE_S;
        config_hsvs[i].lower.v = config.ROOMBA5_LOWER_ORANGE_V;
        config_hsvs[i].upper.h = config.ROOMBA5_UPPER_ORANGE_H;
        config_hsvs[i].upper.s = config.ROOMBA5_UPPER_ORANGE_S;
        config_hsvs[i].upper.v = config.ROOMBA5_UPPER_ORANGE_V;
    } else if (colors[i] == "purple") {
        config_hsvs[i].lower.h = config.ROOMBA5_LOWER_PURPLE_H;
        config_hsvs[i].lower.s = config.ROOMBA5_LOWER_PURPLE_S;
        config_hsvs[i].lower.v = config.ROOMBA5_LOWER_PURPLE_V;
        config_hsvs[i].upper.h = config.ROOMBA5_UPPER_PURPLE_H;
        config_hsvs[i].upper.s = config.ROOMBA5_UPPER_PURPLE_S;
        config_hsvs[i].upper.v = config.ROOMBA5_UPPER_PURPLE_V;
    } else if (colors[i] == "apricot") {
        config_hsvs[i].lower.h = config.ROOMBA5_LOWER_APRICOT_H;
        config_hsvs[i].lower.s = config.ROOMBA5_LOWER_APRICOT_S;
        config_hsvs[i].lower.v = config.ROOMBA5_LOWER_APRICOT_V;
        config_hsvs[i].upper.h = config.ROOMBA5_UPPER_APRICOT_H;
        config_hsvs[i].upper.s = config.ROOMBA5_UPPER_APRICOT_S;
        config_hsvs[i].upper.v = config.ROOMBA5_UPPER_APRICOT_V;
    } else if (colors[i] == "navy") {
        config_hsvs[i].lower.h = config.ROOMBA5_LOWER_NAVY_H;
        config_hsvs[i].lower.s = config.ROOMBA5_LOWER_NAVY_S;
        config_hsvs[i].lower.v = config.ROOMBA5_LOWER_NAVY_V;
        config_hsvs[i].upper.h = config.ROOMBA5_UPPER_NAVY_H;
        config_hsvs[i].upper.s = config.ROOMBA5_UPPER_NAVY_S;
        config_hsvs[i].upper.v = config.ROOMBA5_UPPER_NAVY_V;
    } else if (colors[i] == "olive") {
        config_hsvs[i].lower.h = config.ROOMBA5_LOWER_OLIVE_H;
        config_hsvs[i].lower.s = config.ROOMBA5_LOWER_OLIVE_S;
        config_hsvs[i].lower.v = config.ROOMBA5_LOWER_OLIVE_V;
        config_hsvs[i].upper.h = config.ROOMBA5_UPPER_OLIVE_H;
        config_hsvs[i].upper.s = config.ROOMBA5_UPPER_OLIVE_S;
        config_hsvs[i].upper.v = config.ROOMBA5_UPPER_OLIVE_V;
    } else if (colors[i] == "brown") {
        config_hsvs[i].lower.h = config.ROOMBA5_LOWER_BROWN_H;
        config_hsvs[i].lower.s = config.ROOMBA5_LOWER_BROWN_S;
        config_hsvs[i].lower.v = config.ROOMBA5_LOWER_BROWN_V;
        config_hsvs[i].upper.h = config.ROOMBA5_UPPER_BROWN_H;
        config_hsvs[i].upper.s = config.ROOMBA5_UPPER_BROWN_S;
        config_hsvs[i].upper.v = config.ROOMBA5_UPPER_BROWN_V;
    }
}

inline void set_roomba6_hsv(const std::vector<std::string> &colors, const color_detector_params::HsvConfig &config,
                            int i, std::vector<ThresholdHSV> &config_hsvs) {
    if (colors[i] == "red") {
        config_hsvs[i].lower.h = config.ROOMBA6_LOWER_RED_H;
        config_hsvs[i].lower.s = config.ROOMBA6_LOWER_RED_S;
        config_hsvs[i].lower.v = config.ROOMBA6_LOWER_RED_V;
        config_hsvs[i].upper.h = config.ROOMBA6_UPPER_RED_H;
        config_hsvs[i].upper.s = config.ROOMBA6_UPPER_RED_S;
        config_hsvs[i].upper.v = config.ROOMBA6_UPPER_RED_V;
    } else if (colors[i] == "green") {
        config_hsvs[i].lower.h = config.ROOMBA6_LOWER_GREEN_H;
        config_hsvs[i].lower.s = config.ROOMBA6_LOWER_GREEN_S;
        config_hsvs[i].lower.v = config.ROOMBA6_LOWER_GREEN_V;
        config_hsvs[i].upper.h = config.ROOMBA6_UPPER_GREEN_H;
        config_hsvs[i].upper.s = config.ROOMBA6_UPPER_GREEN_S;
        config_hsvs[i].upper.v = config.ROOMBA6_UPPER_GREEN_V;
    } else if (colors[i] == "blue") {
        config_hsvs[i].lower.h = config.ROOMBA6_LOWER_BLUE_H;
        config_hsvs[i].lower.s = config.ROOMBA6_LOWER_BLUE_S;
        config_hsvs[i].lower.v = config.ROOMBA6_LOWER_BLUE_V;
        config_hsvs[i].upper.h = config.ROOMBA6_UPPER_BLUE_H;
        config_hsvs[i].upper.s = config.ROOMBA6_UPPER_BLUE_S;
        config_hsvs[i].upper.v = config.ROOMBA6_UPPER_BLUE_V;
    } else if (colors[i] == "yellow") {
        config_hsvs[i].lower.h = config.ROOMBA6_LOWER_YELLOW_H;
        config_hsvs[i].lower.s = config.ROOMBA6_LOWER_YELLOW_S;
        config_hsvs[i].lower.v = config.ROOMBA6_LOWER_YELLOW_V;
        config_hsvs[i].upper.h = config.ROOMBA6_UPPER_YELLOW_H;
        config_hsvs[i].upper.s = config.ROOMBA6_UPPER_YELLOW_S;
        config_hsvs[i].upper.v = config.ROOMBA6_UPPER_YELLOW_V;
    } else if (colors[i] == "orange") {
        config_hsvs[i].lower.h = config.ROOMBA6_LOWER_ORANGE_H;
        config_hsvs[i].lower.s = config.ROOMBA6_LOWER_ORANGE_S;
        config_hsvs[i].lower.v = config.ROOMBA6_LOWER_ORANGE_V;
        config_hsvs[i].upper.h = config.ROOMBA6_UPPER_ORANGE_H;
        config_hsvs[i].upper.s = config.ROOMBA6_UPPER_ORANGE_S;
        config_hsvs[i].upper.v = config.ROOMBA6_UPPER_ORANGE_V;
    } else if (colors[i] == "purple") {
        config_hsvs[i].lower.h = config.ROOMBA6_LOWER_PURPLE_H;
        config_hsvs[i].lower.s = config.ROOMBA6_LOWER_PURPLE_S;
        config_hsvs[i].lower.v = config.ROOMBA6_LOWER_PURPLE_V;
        config_hsvs[i].upper.h = config.ROOMBA6_UPPER_PURPLE_H;
        config_hsvs[i].upper.s = config.ROOMBA6_UPPER_PURPLE_S;
        config_hsvs[i].upper.v = config.ROOMBA6_UPPER_PURPLE_V;
    } else if (colors[i] == "apricot") {
        config_hsvs[i].lower.h = config.ROOMBA6_LOWER_APRICOT_H;
        config_hsvs[i].lower.s = config.ROOMBA6_LOWER_APRICOT_S;
        config_hsvs[i].lower.v = config.ROOMBA6_LOWER_APRICOT_V;
        config_hsvs[i].upper.h = config.ROOMBA6_UPPER_APRICOT_H;
        config_hsvs[i].upper.s = config.ROOMBA6_UPPER_APRICOT_S;
        config_hsvs[i].upper.v = config.ROOMBA6_UPPER_APRICOT_V;
    } else if (colors[i] == "navy") {
        config_hsvs[i].lower.h = config.ROOMBA6_LOWER_NAVY_H;
        config_hsvs[i].lower.s = config.ROOMBA6_LOWER_NAVY_S;
        config_hsvs[i].lower.v = config.ROOMBA6_LOWER_NAVY_V;
        config_hsvs[i].upper.h = config.ROOMBA6_UPPER_NAVY_H;
        config_hsvs[i].upper.s = config.ROOMBA6_UPPER_NAVY_S;
        config_hsvs[i].upper.v = config.ROOMBA6_UPPER_NAVY_V;
    } else if (colors[i] == "olive") {
        config_hsvs[i].lower.h = config.ROOMBA6_LOWER_OLIVE_H;
        config_hsvs[i].lower.s = config.ROOMBA6_LOWER_OLIVE_S;
        config_hsvs[i].lower.v = config.ROOMBA6_LOWER_OLIVE_V;
        config_hsvs[i].upper.h = config.ROOMBA6_UPPER_OLIVE_H;
        config_hsvs[i].upper.s = config.ROOMBA6_UPPER_OLIVE_S;
        config_hsvs[i].upper.v = config.ROOMBA6_UPPER_OLIVE_V;
    } else if (colors[i] == "brown") {
        config_hsvs[i].lower.h = config.ROOMBA6_LOWER_BROWN_H;
        config_hsvs[i].lower.s = config.ROOMBA6_LOWER_BROWN_S;
        config_hsvs[i].lower.v = config.ROOMBA6_LOWER_BROWN_V;
        config_hsvs[i].upper.h = config.ROOMBA6_UPPER_BROWN_H;
        config_hsvs[i].upper.s = config.ROOMBA6_UPPER_BROWN_S;
        config_hsvs[i].upper.v = config.ROOMBA6_UPPER_BROWN_V;
    }
}

inline void update_hsv_params(const std::vector<std::string> &colors, const color_detector_params::HsvConfig &config,
                              const std::string roomba, std::vector<ThresholdHSV> &config_hsvs) {
    for (size_t i = 0; i < colors.size(); i++) {
        if (roomba == "roomba1") {
            set_roomba1_hsv(colors, config, i, config_hsvs);
        } else if (roomba == "roomba2") {
            set_roomba2_hsv(colors, config, i, config_hsvs);
        } else if (roomba == "roomba3") {
            set_roomba3_hsv(colors, config, i, config_hsvs);
        } else if (roomba == "roomba4") {
            set_roomba4_hsv(colors, config, i, config_hsvs);
        } else if (roomba == "roomba5") {
            set_roomba5_hsv(colors, config, i, config_hsvs);
        } else if (roomba == "roomba6") {
            set_roomba6_hsv(colors, config, i, config_hsvs);
        } else {
            set_hsv(colors, config, i, config_hsvs);
        }
    }
}

}  // namespace color_detector_params_hsv

#endif  // COLOR_DETECTOR_PARAMS_HSV_H_
