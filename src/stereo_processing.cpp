//
// Created by Dingyi Gu on 2022/6/1.
//

/**
 * @author dingyi
 * @start_date 06/01/2022
 * @modified_date 06/01/2022
 * @stereo_processing
 */


#include <iostream>
#include <cmath>
#include <vector>
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"



#include "../hpps/stereo_processing.h"



StereoProcessing::StereoProcessing() = default;
StereoProcessing::~StereoProcessing() = default;


void StereoProcessing::DepthEstimation() {

    cv::Mat SrcImageL = cv::imread("../dataset/20220512/raw/left0.jpg" , CV_8UC1);
    cv::Mat SrcImageR = cv::imread("../dataset/20220512/raw/right0.jpg", CV_8UC1);

    std::string extrinsic_filename = "../dataset/20220512/result/extrinsics.yml";
    std::string intrinsic_filename = "../dataset/20220512/result/intrinsics.yml";

    cv::Mat rmap[2][2];
    cv::Mat cameraMatrix[2], distCoeffs[2];
    cv::Size imageSize(SrcImageL.size());

    cv::FileStorage fs(intrinsic_filename, cv::FileStorage::READ);

    if (!fs.isOpened())
    {
        std::cerr << "Failed to open file"
                  << "\n";
    }
    cv::Mat M1, D1, M2, D2;
    fs["M1"] >> M1;
    fs["D1"] >> D1;
    fs["M2"] >> M2;
    fs["D2"] >> D2;

    cameraMatrix[0] = M1;
    cameraMatrix[1] = M2;
    distCoeffs[0] = D1;
    distCoeffs[1] = D2;

    fs.open(extrinsic_filename, cv::FileStorage::READ);
    if (!fs.isOpened())
    {
        std::cerr << "Failed to open file"
                  << "\n";
    }
    cv::Mat R, T, R1, R2, P1, P2, Q;
    fs["R"] >> R;
    fs["T"] >> T;
    // fs["R1"] >> R1;
    // fs["R2"] >> R2;
    // fs["P1"] >> P1;
    // fs["P2"] >> P2;
    // fs["Q"] >> Q;

    cv::Size img_size = SrcImageL.size();
    cv::Rect roi1, roi2;
    stereoRectify(M1, D1, M2, D2, img_size, R, T, R1, R2, P1, P2, Q, cv::CALIB_ZERO_DISPARITY, -1, img_size, &roi1, &roi2);
    // Read cameraMatrix[0] ->M1,cameraMatrix[1]->M2, distCoeffs[0] ->D1, distCoeffs[1] -> D2 from intrinsic.yml
    // TODO

    cv::initUndistortRectifyMap(cameraMatrix[0], distCoeffs[0], R1, P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
    cv::initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);

    cv::Mat rimgL, rimgR, cimgL, cimgR;
    cv::remap(SrcImageL, rimgL, rmap[0][0], rmap[0][1], cv::INTER_LINEAR);
    cv::remap(SrcImageR, rimgR, rmap[1][0], rmap[1][1], cv::INTER_LINEAR);
    // cv::cvtColor(rimgL, cimgL, cv::COLOR_GRAY2BGR);
    // cv::cvtColor(rimgR, cimgR, cv::COLOR_GRAY2BGR);

    std::vector<cv::Point2f> MarkerPosL, MarkerPosR;
    cv::Mat ActualPos;
    GetMarkerPos(rimgL, MarkerPosL, 11, 8);
    GetMarkerPos(rimgR, MarkerPosR, 11, 8);
    // Step One: Sparse Points
    // cv::triangularpoints(v1,v2,matrices...)
    cv::triangulatePoints(P1, P2, MarkerPosL, MarkerPosR, ActualPos);
    std::cout << ActualPos.size() << std::endl;
    std::cout << ActualPos.col(0) << std::endl;
    std::cout << ActualPos.col(1) << std::endl;
    std::cout << ActualPos.col(2) << std::endl;

    // Normalized plane from [x y z w] to [x/w y/w z/w 1] and x/w y/w z/w in (cm)
    for (int i = 0; i <= 87; i++)
    {
        for (int j = 0; j <= 3; j++)
        {
            ActualPos.col(i).row(j) = ActualPos.col(i).row(j) / ActualPos.col(i).row(3);
            // std::cout << ActualPos.col(i).row(j) << "\n";
        }
    }
    // One: Error calculation based on feature points =============================
    std::vector<float> diff;
    float diff_xtmp, diff_ytmp, diff_ztmp, diff_tmp, diff_mean, diff_sum;
    diff_sum = 0;
    for (int i = 0; i <= 87 - 1; i++)
    {
        diff_xtmp = ActualPos.at<float>(0, i + 1) - ActualPos.at<float>(0, i);
        diff_ytmp = ActualPos.at<float>(1, i + 1) - ActualPos.at<float>(1, i);
        diff_ztmp = ActualPos.at<float>(2, i + 1) - ActualPos.at<float>(2, i);
        diff_tmp = sqrt(pow(diff_xtmp, 2) + pow(diff_ytmp, 2) + pow(diff_ztmp, 2));


        if (diff_tmp < 2.0)
        {
            std::cout << diff_tmp << "\n";
            diff_sum = diff_sum + diff_tmp;
            diff.push_back(diff_tmp);
        }
    }

    diff_mean = diff_sum / diff.size();
    std::cout << "The mean difference between two corners in mm is: "
              << "\n";
    std::cout << diff_mean*10 << std::endl;

    float err;
    err = diff_mean*10 - 3.0;
    std::cout << "The mean error in (mm) is: " << "\n";
    std::cout << err << "\n";


    // Two: Error Calculation based on Chinshine's Method  =============================
    // Step One: Use plane fitting method to find the plane's parameters
    // Plane fitting(least square): Z = AX+BY+C
    cv::Vec4f v = FitPlane(ActualPos, 88);
    std::cout << "Plane Coefficient is: " << "\n";
    std::cout << "A: " << v[0] << "\n";
    std::cout << "B: " << v[1] << "\n";
    std::cout << "C: " << v[2] << "\n";
    // Step Two: Use plane parameters to estimate all Z axis values again
    // Z = AX+BY+C
    //std::vector<double> zValues;
    cv::Mat xValues = ActualPos.row(0).t(); // Nx1
    cv::Mat yValues = ActualPos.row(1).t(); // Nx1
    cv::Mat cValues = v[2]*cv::Mat::ones(xValues.size(),xValues.type());
    cv::Mat zValues = v[0]*xValues+v[1]*yValues + cValues;

    // Step Three: Calculate mean value of Z values and std deviation of Z values
    cv::Mat zValueSDMat, zValueMMat;
    float zValueMean = cv::mean(zValues).val[0];
    cv::meanStdDev(zValues,zValueMMat, zValueSDMat);
    double zValueStandDev = zValueSDMat.at<double>(0,0);
    double zValueErrorChSh = zValueStandDev*3.0f;
    std::cout << "Z value mean: " << zValueMean << std::endl;
    std::cout << "Z value std dev: " << zValueStandDev << std::endl;
    std::cout << "Z value Error in ChinShine method: " << zValueErrorChSh << std::endl;


}






void StereoProcessing::GetMarkerPos(const cv::Mat RgbImage, std::vector<cv::Point2f> &MarkerPos, const int crow,
                                    const int ccol) {

    std::cout << "Build the marker test example.\n";
    // const int crow(Rows);
    // const int ccol(Cols);

    if (MarkerPos.size() != 0)
    {
        MarkerPos.clear();
    }

    // Detect chess board
    cv::Mat src(RgbImage.clone());
    cv::Size PatSize;
    PatSize.width = crow;
    PatSize.height = ccol;
    if (src.empty())
    {
        std::cout << "input image failures!" << std::endl;
    }
    bool found = findChessboardCorners(src, PatSize, MarkerPos, cv::CALIB_CB_ADAPTIVE_THRESH);
    if (found)
    {
        // find4QuadCornerSubpix(src, MarkerPos, cv::Size(5, 5));
        std::cout << "find corners " << MarkerPos.size() << std::endl;
        drawChessboardCorners(src, PatSize, MarkerPos, found);
        namedWindow("chessboard corners", cv::WINDOW_NORMAL);
        cv::resizeWindow("chessboard corners", 640, 400);
        imshow("chessboard corners", src);
        // cv::waitKey(0);
    }
    else
    {
        std::cout << "find corners failured!" << std::endl;
    }

}





cv::Vec4f StereoProcessing::FitPlane(cv::Mat input_pts, int numpt) {

    cv::Mat pts(numpt, 3, CV_32FC1);
    for (int i = 0; i < numpt; i++)
    {
        pts.at<float>(i, 0) = input_pts.at<float>(0, i);
        pts.at<float>(i, 1) = input_pts.at<float>(1, i);
        pts.at<float>(i, 2) = input_pts.at<float>(2, i);
    }

    double A11 = 0, A12 = 0, A13 = 0;
    double A21 = 0, A22 = 0, A23 = 0;
    double A31 = 0, A32 = 0, A33 = 0;
    double B1 = 0, B2 = 0, B3 = 0;
    for (int i = 0; i < numpt; i++)
    {
        A11 += pts.at<float>(i, 0) * pts.at<float>(i, 0);
        A12 += pts.at<float>(i, 0) * pts.at<float>(i, 1);
        A13 += pts.at<float>(i, 0);
        A21 += pts.at<float>(i, 0) * pts.at<float>(i, 0);
        A22 += pts.at<float>(i, 1) * pts.at<float>(i, 1);
        A23 += pts.at<float>(i, 1);
        A31 += pts.at<float>(i, 0);
        A32 += pts.at<float>(i, 1);
        B1 += pts.at<float>(i, 0) * pts.at<float>(i, 2);
        B2 += pts.at<float>(i, 1) * pts.at<float>(i, 2);
        B3 += pts.at<float>(i, 2);
    }

    A33 = numpt;

    cv::Mat A(3, 3, CV_32FC1);
    A.at<float>(0, 0) = A11;
    A.at<float>(0, 1) = A12;
    A.at<float>(0, 2) = A13;
    A.at<float>(1, 0) = A21;
    A.at<float>(1, 1) = A22;
    A.at<float>(1, 2) = A23;
    A.at<float>(2, 0) = A31;
    A.at<float>(2, 1) = A32;
    A.at<float>(2, 2) = A33;

    cv::Mat B(3, 1, CV_32FC1);
    B.at<float>(0, 0) = B1;
    B.at<float>(1, 0) = B2;
    B.at<float>(2, 0) = B3;
    cv::Mat X;
    X = A.inv() * B;
    cv::Vec4f v;
    v[0] = X.at<float>(0, 0);
    v[1] = X.at<float>(1, 0);
    v[2] = X.at<float>(2, 0);

    return v;

}