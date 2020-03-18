#include <opencv2/opencv.hpp>
#include <string>
#include <iostream>
#include "calibration.h"
#include "undistort.h"
#include "outermatch.h"

using namespace std;
using namespace cv;

int main()
{
    string patternImgPath="/home/songsong/Documents/private/Graduation design/codes/camera-calibration/data/pattern/";
    string calibResultPath="/home/songsong/Documents/private/Graduation design/codes/camera-calibration/data/results/";
//    string srcImg_left="/home/songsong/Documents/private/Graduation design/codes/camera-calibration/data/srcImg/image_left.jpg";
//    string srcImg_right="/home/songsong/Documents/private/Graduation design/codes/camera-calibration/data/srcImg/image_right.jpg";
    string srcImg_left="/home/songsong/Documents/private/Graduation design/codes/camera-calibration/data/srcImg/1.jpg";
    string srcImg_right="/home/songsong/Documents/private/Graduation design/codes/camera-calibration/data/srcImg/2.jpg";
    Size boardSize=Size(7,7);
    CCalibration calibration(patternImgPath, calibResultPath, boardSize);
    calibration.run();
    CUndistort undistort1(srcImg_left, calibResultPath);
    undistort1.run();

    CUndistort undistort2(srcImg_right, calibResultPath);
    undistort2.run();

    Outermatch om1(srcImg_left,srcImg_right,calibResultPath);


}