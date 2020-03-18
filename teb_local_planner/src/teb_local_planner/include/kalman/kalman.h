
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
    kalman_filter(Point2f tp);
    kalman_filter();
    ~kalman_filter();
    void update(Point2f pt);
    void draw(Mat &image);
    Point2f get_predict_pt();
    Point2f predict_kt(int kt);

    KalmanFilter KF;
    Mat measurement;
   // void output(Mat &tp);

private:

    Point2f measure_pt;
    Point2f predict_pt;
};

Mat convertTo3Channels( Mat &binImg);





#endif //TEST_KALMAN_H
