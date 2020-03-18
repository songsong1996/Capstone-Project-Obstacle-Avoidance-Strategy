//
// Created by songsong on 19-3-17.
//

#ifndef CAMERA_CALIBRATE_OUTERMATCH_H
#define CAMERA_CALIBRATE_OUTERMATCH_H

#include <opencv2/opencv.hpp>
#include <string>
#include <iostream>
using namespace std;
using namespace cv;
class Outermatch
{
public:
    string imageLeftPath;
    string imageRightPath;
    string cameraMatPath;
    void readCameraMat();
    Mat RectifyImage();
    Mat SGBM();
    Outermatch(string leftpath,string rightpath,string matpath)
    {
        imageLeftPath=leftpath;
        imageRightPath=rightpath;
        cameraMatPath=matpath;
    };
    ~Outermatch(){};

private:
    Mat camK;
    Mat camDiscoeff;
    Mat recImgL,recImgR;
    Mat image_left,image_right;
};


#endif //CAMERA_CALIBRATE_OUTERMATCH_H
