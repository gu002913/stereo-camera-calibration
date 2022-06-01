//
// Created by Dingyi Gu on 2022/6/1.
//

#ifndef STEREO_CAMERA_CPP_STEREO_PROCESSING_H
#define STEREO_CAMERA_CPP_STEREO_PROCESSING_H

/**
 * @author dingyi
 * @start_date 06/01/2022
 * @modified_date 06/01/2022
 * @stereo_processing
 */


#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>


class StereoProcessing{
public:

    StereoProcessing();
    ~StereoProcessing();

    void DepthEstimation();

private:

    void GetMarkerPos(const cv::Mat RgbImage, std::vector<cv::Point2f>& MarkerPos, const int crow= 8, const int ccol = 11);
    cv::Vec4f FitPlane(cv::Mat input_pts, int numpt);


};






#endif //STEREO_CAMERA_CPP_STEREO_PROCESSING_H
