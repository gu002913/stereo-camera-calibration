//
// Created by Dingyi Gu on 2022/1/10.
//


#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <cmath>
#include <vector>
#include <string>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <iterator>
#include <cstdio>
#include <cstdlib>
#include <cctype>



#include "../hpps/calibrate_stereo.h"


/**
 * @author dingyi
 * @start_date 01/10/2022
 * @modified_date 05/31/2022
 * @stereo_calibration
 */


calibrate_stereo::calibrate_stereo() {}
calibrate_stereo::~calibrate_stereo() {}


int calibrate_stereo::CalibrationExecutable(int argc, char** argv) {

    cv::Size boardSize;
    std::string imagelistfn;
    std::string filePath = "/Users/gudingyi/Desktop/stereo-camera-calibration/calib_data/20220216_dataset/5cm_w_light/raw/";
    bool showRectified;
    cv::CommandLineParser parser(argc, argv, "{w|11|}{h|8|}{s|0.3|}{nr||}{help||}{@input|" + filePath + "stereo_calib.xml|}");
    if (parser.has("help"))
        return print_help(argv);
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
    bool ok = readStringList(imagelistfn, imagelist);
    if(!ok || imagelist.empty())
    {
        std::cout << "can not open " << imagelistfn << " or the string list is empty" << std::endl;
        return print_help(argv);
    }

    StereoCalib(imagelist, boardSize, squareSize, false, true, showRectified);

    return 0;
}









int calibrate_stereo::print_help(char **argv) {
    std::cout <<
              " Given a list of chessboard images, the number of corners (nx, ny)\n"
              " on the chessboards, and a flag: useCalibrated for \n"
              "   calibrated (0) or\n"
              "   uncalibrated \n"
              "     (1: use stereoCalibrate(), 2: compute fundamental\n"
              "         matrix separately) stereo. \n"
              " Calibrate the cameras and display the\n"
              " rectified results along with the computed disparity images.   \n" << std::endl;
    std::cout << "Usage:\n " << argv[0]
              << " -w=<board_width default=9> -h=<board_height default=6> -s=<square_size default=1.0> <image list XML/YML file default=stereo_calib.xml>\n"
              << std::endl;

    return 0;
}


void calibrate_stereo::StereoCalib(const std::vector<std::string> &imagelist, cv::Size boardSize, float squareSize,
                                   bool displayCorners, bool useCalibrated, bool showRectified) {
    if( imagelist.size() % 2 != 0 )
    {
        std::cout << "Error: the image list contains odd (non-even) number of elements\n";
        return;
    }

    const int maxScale = 2;
    // ARRAY AND VECTOR STORAGE:

    std::vector<std::vector<cv::Point2f>> imagePoints[2];
    std::vector<std::vector<cv::Point3f>> objectPoints;
    cv::Size imageSize;

    int i, j, k, nimages = (int)imagelist.size()/2;

    imagePoints[0].resize(nimages);
    imagePoints[1].resize(nimages);
    std::vector<std::string> goodImageList;

    for( i = j = 0; i < nimages; i++ )
    {
        for( k = 0; k < 2; k++ )
        {
            const std::string &filename = imagelist[i*2+k];
            cv::Mat img = cv::imread(filename, 0);
            if(img.empty())
                break;
            if( imageSize == cv::Size() )
                imageSize = img.size();
            else if( img.size() != imageSize )
            {
                std::cout << "The image " << filename << " has the size different from the first image size. Skipping the pair\n";
                break;
            }
            bool found = false;
            std::vector<cv::Point2f>& corners = imagePoints[k][j];
            for( int scale = 1; scale <= maxScale; scale++ )
            {
                cv::Mat timg;
                if( scale == 1 )
                    timg = img;
                else
                    resize(img, timg, cv::Size(), scale, scale, cv::INTER_LINEAR_EXACT);
                found = findChessboardCorners(timg, boardSize, corners,
                                              cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
                if( found )
                {
                    if( scale > 1 )
                    {
                        cv::Mat cornersMat(corners);
                        cornersMat *= 1./scale;
                    }
                    break;
                }
            }
            if( displayCorners )
            {
                std::cout << filename << std::endl;
                cv::Mat cimg, cimg1;
                cvtColor(img, cimg, cv::COLOR_GRAY2BGR);
                drawChessboardCorners(cimg, boardSize, corners, found);
                double sf = 640./MAX(img.rows, img.cols);
                resize(cimg, cimg1, cv::Size(), sf, sf, cv::INTER_LINEAR_EXACT);
                imshow("corners", cimg1);
                char c = (char) cv::waitKey(500);
                if( c == 27 || c == 'q' || c == 'Q' ) //Allow ESC to quit
                    exit(-1);
            }
            else
                putchar('.');
            if( !found )
                break;
            cornerSubPix(img, corners, cv::Size(11,11), cv::Size(-1,-1),
                         cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS,
                                      30, 0.01));
        }
        if( k == 2 )
        {
            goodImageList.push_back(imagelist[i*2]);
            goodImageList.push_back(imagelist[i*2+1]);
            j++;
        }
    }
    std::cout << j << " pairs have been successfully detected.\n";
    nimages = j;
    if( nimages < 2 )
    {
        std::cout << "Error: too little pairs to run the calibration\n";
        return;
    }

    imagePoints[0].resize(nimages);
    imagePoints[1].resize(nimages);
    objectPoints.resize(nimages);

    for( i = 0; i < nimages; i++ )
    {
        for( j = 0; j < boardSize.height; j++ )
            for( k = 0; k < boardSize.width; k++ )
                objectPoints[i].push_back(cv::Point3f(k*squareSize, j*squareSize, 0));
    }

    std::cout << "Running stereo calibration ...\n";

    cv::Mat cameraMatrix[2], distCoeffs[2];
    cameraMatrix[0] = initCameraMatrix2D(objectPoints,imagePoints[0],imageSize,0);
    cameraMatrix[1] = initCameraMatrix2D(objectPoints,imagePoints[1],imageSize,0);
    cv::Mat R, T, E, F;

    double rms = stereoCalibrate(objectPoints, imagePoints[0], imagePoints[1],
                                 cameraMatrix[0], distCoeffs[0],
                                 cameraMatrix[1], distCoeffs[1],
                                 imageSize, R, T, E, F,
                                 cv::CALIB_FIX_ASPECT_RATIO +
                                 cv::CALIB_ZERO_TANGENT_DIST +
                                 cv::CALIB_USE_INTRINSIC_GUESS +
                                 cv::CALIB_SAME_FOCAL_LENGTH +
                                 cv::CALIB_RATIONAL_MODEL +
                                 cv::CALIB_FIX_K3 + cv::CALIB_FIX_K4 + cv::CALIB_FIX_K5,
                                 cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 100, 1e-5) );
    std::cout << "done with RMS error=" << rms << std::endl;

// CALIBRATION QUALITY CHECK
// because the output fundamental matrix implicitly
// includes all the output information,
// we can check the quality of calibration using the
// epipolar geometry constraint: m2^t*F*m1=0
    double err = 0;
    int npoints = 0;
    std::vector<cv::Vec3f> lines[2];
    for( i = 0; i < nimages; i++ )
    {
        int npt = (int)imagePoints[0][i].size();
        cv::Mat imgpt[2];
        for( k = 0; k < 2; k++ )
        {
            imgpt[k] = cv::Mat(imagePoints[k][i]);
            undistortPoints(imgpt[k], imgpt[k], cameraMatrix[k], distCoeffs[k], cv::Mat(), cameraMatrix[k]);
            computeCorrespondEpilines(imgpt[k], k+1, F, lines[k]);
        }
        for( j = 0; j < npt; j++ )
        {
            double errij = fabs(imagePoints[0][i][j].x*lines[1][j][0] +
                                imagePoints[0][i][j].y*lines[1][j][1] + lines[1][j][2]) +
                           fabs(imagePoints[1][i][j].x*lines[0][j][0] +
                                imagePoints[1][i][j].y*lines[0][j][1] + lines[0][j][2]);
            err += errij;
        }
        npoints += npt;
    }
    std::cout << "average epipolar err = " <<  err/npoints << std::endl;

    // save intrinsic parameters
    cv::FileStorage fs("/Users/gudingyi/Desktop/stereo-camera-calibration/calib_data/20220216_dataset/5cm_w_light/result/intrinsics.yml", cv::FileStorage::WRITE);
    if( fs.isOpened() )
    {
        fs << "M1" << cameraMatrix[0] << "D1" << distCoeffs[0] <<
           "M2" << cameraMatrix[1] << "D2" << distCoeffs[1];
        fs.release();
    }
    else
        std::cout << "Error: can not save the intrinsic parameters\n";

    cv::Mat R1, R2, P1, P2, Q;
    cv::Rect validRoi[2];

    stereoRectify(cameraMatrix[0], distCoeffs[0],
                  cameraMatrix[1], distCoeffs[1],
                  imageSize, R, T, R1, R2, P1, P2, Q,
                  cv::CALIB_ZERO_DISPARITY, 1, imageSize, &validRoi[0], &validRoi[1]);

    fs.open("/Users/gudingyi/Desktop/stereo-camera-calibration/calib_data/20220216_dataset/5cm_w_light/result/extrinsics.yml", cv::FileStorage::WRITE);
    if( fs.isOpened() )
    {
        fs << "R" << R << "T" << T << "R1" << R1 << "R2" << R2 << "P1" << P1 << "P2" << P2 << "Q" << Q;
        fs.release();
    }
    else
        std::cout << "Error: can not save the extrinsic parameters\n";

    // OpenCV can handle left-right
    // or up-down camera arrangements
    bool isVerticalStereo = fabs(P2.at<double>(1, 3)) > fabs(P2.at<double>(0, 3));

// COMPUTE AND DISPLAY RECTIFICATION
    if( !showRectified )
        return;

    cv::Mat rmap[2][2];
// IF BY CALIBRATED (BOUGUET'S METHOD)
    if( useCalibrated )
    {
        // we already computed everything
    }
// OR ELSE HARTLEY'S METHOD
    else
        // use intrinsic parameters of each camera, but
        // compute the rectification transformation directly
        // from the fundamental matrix
    {
        std::vector<cv::Point2f> allimgpt[2];
        for( k = 0; k < 2; k++ )
        {
            for( i = 0; i < nimages; i++ )
                std::copy(imagePoints[k][i].begin(), imagePoints[k][i].end(), back_inserter(allimgpt[k]));
        }
        F = findFundamentalMat(cv::Mat(allimgpt[0]), cv::Mat(allimgpt[1]), cv::FM_8POINT, 0, 0);
        cv::Mat H1, H2;
        stereoRectifyUncalibrated(cv::Mat(allimgpt[0]), cv::Mat(allimgpt[1]), F, imageSize, H1, H2, 3);

        R1 = cameraMatrix[0].inv()*H1*cameraMatrix[0];
        R2 = cameraMatrix[1].inv()*H2*cameraMatrix[1];
        P1 = cameraMatrix[0];
        P2 = cameraMatrix[1];
    }

    //Precompute maps for cv::remap()
    initUndistortRectifyMap(cameraMatrix[0], distCoeffs[0], R1, P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
    initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);

    cv::Mat canvas;
    double sf;
    int w, h;
    if( !isVerticalStereo )
    {
        sf = 600./MAX(imageSize.width, imageSize.height);
        w = cvRound(imageSize.width*sf);
        h = cvRound(imageSize.height*sf);
        canvas.create(h, w*2, CV_8UC3);
    }
    else
    {
        sf = 300./MAX(imageSize.width, imageSize.height);
        w = cvRound(imageSize.width*sf);
        h = cvRound(imageSize.height*sf);
        canvas.create(h*2, w, CV_8UC3);
    }

    for( i = 0; i < nimages; i++ )
    {
        for( k = 0; k < 2; k++ )
        {
            cv::Mat img = cv::imread(goodImageList[i*2+k], 0), rimg, cimg;
            remap(img, rimg, rmap[k][0], rmap[k][1], cv::INTER_LINEAR);
            cvtColor(rimg, cimg, cv::COLOR_GRAY2BGR);
            cv::Mat canvasPart = !isVerticalStereo ? canvas(cv::Rect(w*k, 0, w, h)) : canvas(cv::Rect(0, h*k, w, h));
            resize(cimg, canvasPart, canvasPart.size(), 0, 0, cv::INTER_AREA);
            if( useCalibrated )
            {
                cv::Rect vroi(cvRound(validRoi[k].x*sf), cvRound(validRoi[k].y*sf),
                          cvRound(validRoi[k].width*sf), cvRound(validRoi[k].height*sf));
                rectangle(canvasPart, vroi, cv::Scalar(0,0,255), 3, 8);
            }
        }

        if( !isVerticalStereo )
            for( j = 0; j < canvas.rows; j += 16 )
                line(canvas, cv::Point(0, j), cv::Point(canvas.cols, j), cv::Scalar(0, 255, 0), 1, 8);
        else
            for( j = 0; j < canvas.cols; j += 16 )
                line(canvas, cv::Point(j, 0), cv::Point(j, canvas.rows), cv::Scalar(0, 255, 0), 1, 8);
        imshow("rectified", canvas);
        char c = (char) cv::waitKey();
        if( c == 27 || c == 'q' || c == 'Q' )
            break;
    }

}


bool calibrate_stereo::readStringList(const std::string &filename, std::vector<std::string> &l) {
    l.resize(0);
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if( !fs.isOpened() )
        return false;
    cv::FileNode n = fs.getFirstTopLevelNode();
    if( n.type() != cv::FileNode::SEQ )
        return false;
    cv::FileNodeIterator it = n.begin(), it_end = n.end();
    for( ; it != it_end; ++it )
        l.push_back((std::string)*it);

    return true;
}


