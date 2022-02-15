## **Stereo Camera Calibration Using OpenCV C++**

***

This repository contains some sources to calibrate the intrinsics of individual cameras and also the extrinsics of a stereo pair. 

### **Dependencies**

* OpenCV

```js
% brew install opencv
```

### **Compilation**

Compile all the files using the following commands.

```js
% mkdir cmake-build-debug
% cd cmake-build-debug
% cmake ..
% make
```

Make sure you are in the `cmake-build-debug` folder to run the exeutables.

```js
% ./stereo-camera-calibration
```

### **Data Analysis(calib_data)**

| File name                                            | Description                                                                                                                                                                                   |
| ---------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| calib_data/20220208_4cm/ opencv_algo/raw/left_xx.jpg | **4cm** means the camera baseline and `left_xx.jpg` or `right_xx.jpg` refers to the sequence of the captured image from the stereo pair.                                                      |
| ../raw/stereo_calib.xml                              | this `xml` file determines the sequence of the raw images.                                                                                                                                    |
| ../result/extrinsics.yml                             | this `yml` file denotes the extrinsic parameters including **rotational matrix**, **translational matrix**, the decomposition of the **homography matrix** (reprojection) of the stereo pair. |
| ../result/intrinsics.yml                             | this `yml` file denotes the intrinsic parameters including each of the **intrinsic matrix** and **distortion coefficients** (either rational or tangential).                                  |
| ../result/rectified.png                              | this `png` file presents the **epipolar rectification**.                                                                                                                                      |
| ../result/error_metrics.png                          | this `png` file presents the **RMS error** and **average epipolar error** (per pixel).                                                                                                        |

### **How to modify the filepath**

input path in `main.cpp`:

```js
int main(int argc, char** argv){

    calibrate_stereo calibrateStereo;
    cv::Size boardSize;
    std::string imagelistfn;
    std::string filePath = "/Desktop/stereo-camera-calibration/calib_data/20220208_4cm/opencv_algo/raw/";
    bool showRectified;
    cv::CommandLineParser parser(argc, argv, "{w|11|}{h|8|}{s|0.3|}{nr||}{help||}{@input|" + filePath + "stereo_calib.xml|}");
...
return 0;
}
```

output path in `calibrate_stereo.cpp`:

```js
void calibrate_stereo::StereoCalib(const std::vector<std::string> &imagelist, cv::Size boardSize, float squareSize,
                                   bool displayCorners, bool useCalibrated, bool showRectified) {
...
    // save intrinsic parameters
    cv::FileStorage fs("/Desktop/stereo-camera-calibration/calib_data/20220208_4cm/opencv_algo/result/intrinsics.yml", cv::FileStorage::WRITE);

...
    stereoRectify(cameraMatrix[0], distCoeffs[0],
                  cameraMatrix[1], distCoeffs[1],
                  imageSize, R, T, R1, R2, P1, P2, Q,
                  cv::CALIB_ZERO_DISPARITY, 1, imageSize, &validRoi[0], &validRoi[1]);
    fs.open("/Desktop/stereo-camera-calibration/calib_data/20220208_4cm/opencv_algo/result/extrinsics.yml", cv::FileStorage::WRITE);
...
}
```

### **References**

***

<a id="1">[1]</a> 
Bradski, G., 2000. 
The OpenCV Library. 
*Dr. Dobb&#x27;s Journal of Software Tools*.

<a id="2">[2]</a> 
Zhang, Z., 2000.
A flexible new technique for camera calibration. 
*IEEE Transactions on pattern analysis and machine intelligence*, *22*(11), pp.1330-1334.

<a id="3">[3]</a> 
Loop, C.T. and Zhang, Z., 1999, January. 
Computing Rectifying Homographies for Stereo Vision. 
In *CVPR*.
