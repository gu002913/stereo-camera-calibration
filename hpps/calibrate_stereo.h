//
// Created by Dingyi Gu on 2022/1/10.
//

#ifndef STEREO_CAMERA_CALIBRATION_CalibrateStereo_H
#define STEREO_CAMERA_CALIBRATION_CalibrateStereo_H

/**
 * @author dingyi
 * @start_date 01/10/2022
 * @modified_date 05/31/2022
 * @stereo_calibration
 */



#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

class CalibrateStereo {

public:

    CalibrateStereo();
    ~CalibrateStereo();


    int CalibrationExecutable(int argc, char** argv);




private:

    int PrintHelp(char** argv);

    void StereoCalib(const std::vector<std::string> &imagelist, cv::Size boardSize, float squareSize,
                     bool displayCorners = false, bool useCalibrated = true, bool showRectified = true);

    bool ReadStringList( const std::string &filename, std::vector<std::string> &l );
};


#endif //STEREO_CAMERA_CALIBRATION_CalibrateStereo_H
