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


/**
 * @author dingyi
 * @start_date 01/10/2022
 * @modified_date 05/31/2022
 * @stereo_calibration
 */

int main(int argc, char** argv){


    calibrate_stereo calibrateStereo;
    calibrateStereo.CalibrationExecutable(argc, argv);



    return 0;
}