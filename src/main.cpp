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


#include "../hpps/calibrate_stereo.h"
#include "../hpps/stereo_processing.h"


/**
 * @author dingyi
 * @start_date 01/10/2022
 * @modified_date 05/31/2022
 * @stereo_calibration
 */


int main(int argc, char** argv){

    // Stereo Calibration ==============================================================================
//    CalibrateStereo calibrateStereo;
//    calibrateStereo.CalibrationExecutable(argc, argv);
    // =================================================================================================


    // Depth Analysis ==================================================================================
    StereoProcessing stereoProcessing;
    stereoProcessing.DepthEstimation();
    // =================================================================================================




    return 0;
}



