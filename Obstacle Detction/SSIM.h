//
// Created by songsong on 19-6-7.
//

#ifndef TEST_SSIM_H
#define TEST_SSIM_H


#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<iostream>

using namespace std;
using namespace cv;

Scalar getMSSIM( const Mat& i1, const Mat& i2);
double SSIM(Mat im1,Mat im2);



#endif //TEST_SSIM_H
