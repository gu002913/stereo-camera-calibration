//
// Created by Dingyi Gu on 2022/1/10.
//

#ifndef STEREO_CAMERA_CALIBRATION_CALIBRATE_STEREO_H
#define STEREO_CAMERA_CALIBRATION_CALIBRATE_STEREO_H


#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

class calibrate_stereo {

public:

    calibrate_stereo();

    static int print_help(char** argv);

    static void StereoCalib(const std::vector<std::string> &imagelist, cv::Size boardSize, float squareSize,
                            bool displayCorners = false, bool useCalibrated = true, bool showRectified = true);

    static bool readStringList( const std::string &filename, std::vector<std::string> &l );



private:

};


#endif //STEREO_CAMERA_CALIBRATION_CALIBRATE_STEREO_H
