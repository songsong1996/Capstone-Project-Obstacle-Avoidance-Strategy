
#ifndef TEST_KALMAN_H
#define TEST_KALMAN_H


#include "opencv2/video/tracking.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdio.h>
using namespace cv;
using namespace std;

const int winHeight=600;
const int winWidth=800;


class kalman_filter
{
public:
    kalman_filter(Point tp);
    kalman_filter();
    ~kalman_filter();
    void update(Point pt);
    void draw(Mat &image);
    Point get_predict_pt();
   // void output(Mat &tp);

private:
    KalmanFilter KF;
    Point measure_pt;
    Mat measurement;
    Point predict_pt;
};

Mat convertTo3Channels( Mat &binImg);





#endif //TEST_KALMAN_H
