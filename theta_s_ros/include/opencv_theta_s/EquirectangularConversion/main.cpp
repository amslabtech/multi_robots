//
//  main.cpp
//  EquirectangularConversion
//
//  Created by Kozo Komiya on 2018/10/18.
//  Copyright © 2018 Kozo Komiya. All rights reserved.
//

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "ThetaConversion.hpp"

int main(int argc, const char* argv[]) {
    if (argc < 3) {
        std::cerr << "Equirectangular conversion for Theta S" << std::endl;
        std::cerr << '\n';
        std::cerr << "Usage:" << std::endl;
        std::cerr << "$ ec_thetas <input file> <output file>" << std::endl;
        return -1;
    }
    std::string input_file = argv[1];
    std::string output_file = argv[2];
    std::cout << "Input file  : " << input_file << std::endl;
    std::cout << "Output file : " << output_file << std::endl;

    cv::VideoCapture cap(input_file);
    if (!cap.isOpened()) {
        std::cerr << "Error: Input file can't open. " << input_file << std::endl;
        return -1;
    }
    // width
    int width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
    // height
    int height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    // number of frames
    int count = cap.get(cv::CAP_PROP_FRAME_COUNT);
    // fps
    double fps = cap.get(cv::CAP_PROP_FPS);

    std::cout << "size = " << width << "x" << height << std::endl;
    std::cout << "frame count = " << count << std::endl;
    std::cout << "fps = " << fps << std::endl;

    cv::Size size(width, height);
    int fourcc = cv::VideoWriter::fourcc('m', 'p', '4', 'v');  // .mp4
    cv::VideoWriter writer(output_file, fourcc, fps, size);

    if (!writer.isOpened()) {
        std::cerr << "Error: output file can't open. " << output_file << std::endl;
        return -1;
    }

    cv::Mat mat;
    ThetaConversion theta(width, height);
    for (int i = 0;; i++) {
        cap >> mat;
        if (mat.empty()) break;
        theta.doConversion(mat);
        //        cv::imshow("image", frame);
        //        if (cv::waitKey(1) >= 0) break;
        writer << mat;
        if (i % 30 == 0) std::cout << '.' << std::flush;
    }
    std::cout << '\n';
    return 0;
}
