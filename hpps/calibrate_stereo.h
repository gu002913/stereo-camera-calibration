//
// Created by Dingyi Gu on 2022/1/10.
//

#ifndef STEREO_CAMERA_CALIBRATION_CALIBRATE_STEREO_H
#define STEREO_CAMERA_CALIBRATION_CALIBRATE_STEREO_H

/**
 * @author dingyi
 * @start_date 01/10/2022
 * @modified_date 05/31/2022
 * @stereo_calibration
 */



#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

class calibrate_stereo {

public:

    calibrate_stereo();
    ~calibrate_stereo();


    int CalibrationExecutable(int argc, char** argv);




private:

    int print_help(char** argv);

    void StereoCalib(const std::vector<std::string> &imagelist, cv::Size boardSize, float squareSize,
                     bool displayCorners = false, bool useCalibrated = true, bool showRectified = true);

    bool readStringList( const std::string &filename, std::vector<std::string> &l );
};


#endif //STEREO_CAMERA_CALIBRATION_CALIBRATE_STEREO_H
