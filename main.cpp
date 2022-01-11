//
// Created by Dingyi Gu on 2022/1/11.
//

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <cmath>
#include <vector>
#include <iostream>
#include <fstream>
#include <iterator>




#include "calibrate_stereo.h"


/**
 * @author dingyi
 * @start_date 01/10/2022
 * @modified_date 01/10/2022
 * @stereo_calibration
 */

int main(int argc, char** argv){

    calibrate_stereo calibrateStereo;
    cv::Size boardSize;
    std::string imagelistfn;
    std::string filePath = "/Users/gudingyi/Desktop/stereo-camera-calibration/calib_data/1/opencv_algo/raw/";
    bool showRectified;
    cv::CommandLineParser parser(argc, argv, "{w|11|}{h|8|}{s|0.3|}{nr||}{help||}{@input|" + filePath + "stereo_calib.xml|}");
    if (parser.has("help"))
        return calibrateStereo.print_help(argv);
    showRectified = !parser.has("nr");
    imagelistfn = cv::samples::findFile(parser.get<std::string>("@input"));
    boardSize.width = parser.get<int>("w");
    boardSize.height = parser.get<int>("h");
    float squareSize = parser.get<float>("s");
    if (!parser.check())
    {
        parser.printErrors();
        return 1;
    }
    std::vector<std::string> imagelist;
    bool ok = calibrateStereo.readStringList(imagelistfn, imagelist);
    if(!ok || imagelist.empty())
    {
        std::cout << "can not open " << imagelistfn << " or the string list is empty" << std::endl;
        return calibrateStereo.print_help(argv);
    }

    calibrateStereo.StereoCalib(imagelist, boardSize, squareSize, false, true, showRectified);



    return 0;
}