//
// Created by songsong on 19-6-7.
//

#ifndef TEST_OPTICALFLOW_H
#define TEST_OPTICALFLOW_H

#include <iostream>
#include "opencv2/video/tracking.hpp"
#include "opencv2/highgui/highgui.hpp"
using namespace std;
using namespace cv;

Mat filter(Mat img,Mat ROI);
Mat optical_flow(Mat previous,Mat present,Mat depth1,Mat depth2);

#endif //TEST_OPTICALFLOW_H
