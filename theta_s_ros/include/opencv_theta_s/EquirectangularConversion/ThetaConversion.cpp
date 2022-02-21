//
//  ThetaConversion.cpp
//  EquirectangularConversion
//
//  Created by Kozo Komiya on 2018/10/18.
//  Copyright © 2018 Kozo Komiya. All rights reserved.
//

#include "ThetaConversion.hpp"

// Constructor
ThetaConversion::ThetaConversion(int _w, int _h) : shift(0) {
    cols = _w;
    rows = _h;
    map_x = cv::Mat(cv::Size(cols, rows), CV_32FC1);
    map_y = cv::Mat(cv::Size(cols, rows), CV_32FC1);
    makeMap();
}

// Make mapping table for equirectangular conversion
void ThetaConversion::makeMap() {
    float dst_y = static_cast<float>(cols) / 2;
    float src_cx = static_cast<float>(cols) / 4;
    float src_cy = static_cast<float>(cols) / 4;
    float src_r = 0.884 * static_cast<float>(cols) / 4;
    float src_rx = src_r * 1.00;  // 7.11
    float src_ry = src_r * 1.00;  // 7.11
    float src_cx2 = cols - src_cx;

    // make mapping table
    for (int y = 0; y < dst_y; y++) {
        for (int x = 0; x < cols; x++) {
            float ph1 = M_PI * x / dst_y;
            float th1 = M_PI * y / dst_y;

            float x1 = sin(th1) * cos(ph1);
            float y1 = sin(th1) * sin(ph1);
            float z = cos(th1);

            float ph2 = acos(-x1);
            float th2 = (y1 >= 0 ? 1 : -1) * acos(-z / sqrt(y1 * y1 + z * z));

            float r0;
            if (ph2 < M_PI / 2) {
                r0 = ph2 / (M_PI / 2);  // Equidistant projection
                // r0 = tan(ph2 / 2);               // Stereographic projection
                map_x.at<float>(y, x) = src_rx * r0 * cos(M_PI - th2) + src_cx2;
                map_y.at<float>(y, x) = src_ry * r0 * sin(M_PI - th2) + src_cy;
            } else {
                r0 = (M_PI - ph2) / (M_PI / 2);  // Equidistant projection
                // r0 = tan((M_PI - ph2) / 2);      // Stereographic projection
                map_x.at<float>(y, x) = src_rx * r0 * cos(th2) + src_cx;
                map_y.at<float>(y, x) = src_ry * r0 * sin(th2) + src_cy;
            }
        }
    }
}

void ThetaConversion::doConversion(cv::Mat &mat) {
    equirectangularConversion(mat);
    antiRotate(mat);
    // overlaySizeInfo(mat);
}

void ThetaConversion::overlaySizeInfo(cv::Mat &mat) {
    std::string s = "Size: " + std::to_string(cols) + "x" + std::to_string(rows);
    cv::putText(mat, s, cv::Point(20, 80), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(20, 230, 20), 2, cv::LINE_AA);
}

void ThetaConversion::equirectangularConversion(cv::Mat &mat) {
    cv::Mat buf = cv::Mat(mat.size(), mat.type());
    cv::remap(mat, buf, map_x, map_y, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
    buf.copyTo(mat);
}

void ThetaConversion::antiRotate(cv::Mat &mat) {
    shift += diffRotate(mat);
    cv::Mat buf;

    if (shift >= cols) shift -= cols;
    if (shift < 0) shift += cols;
    if (shift != 0) {
        cv::Mat buf;
        cv::Rect r1(cv::Point(0, 0), cv::Size(shift, rows));
        cv::Rect r2(cv::Point(shift, 0), cv::Size(cols - shift, rows));
        cv::hconcat(mat(r2), mat(r1), buf);
        buf.copyTo(mat);
    }
}

int ThetaConversion::diffRotate(cv::Mat &mat) {
    const int y1 = rows / 2;
    cv::Rect r1(0, y1, cols, 1);
    cv::Mat l1(mat, r1);
    cv::Mat m1;
    cv::cvtColor(l1, m1, cv::COLOR_RGB2GRAY);
    int ret = 0;

    if (prev.cols == cols) {
        int w0 = cols / 12;
        cv::Rect rs0(0, 0, w0, 1);
        cv::Mat t1;
        cv::hconcat(prev, prev(rs0), t1);
        cv::Rect rs1(cols - w0, 0, w0, 1);
        cv::Mat b1;
        cv::hconcat(prev(rs1), t1, b1);
        cv::Mat result1;
        cv::matchTemplate(b1, m1, result1, cv::TM_CCOEFF);
        cv::Point p1;
        double v1;
        minMaxLoc(result1, NULL, &v1, NULL, &p1);
        ret = w0 - p1.x;
    }

    m1.copyTo(prev);
    return ret;
}
