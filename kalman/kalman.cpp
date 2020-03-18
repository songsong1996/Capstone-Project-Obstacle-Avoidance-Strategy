
#include"kalman.h"
#include "opencv2/video/tracking.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdio.h>


using namespace cv;
using namespace std;




kalman_filter::kalman_filter()
{
    int winHeight=600;
    int winWidth=800;
    RNG rng;
    //1.kalman filter setup
    const int stateNum=4;                                      //状态值4×1向量(x,y,△x,△y)
    const int measureNum=2;                                    //测量值2×1向量(x,y)
    KF=KalmanFilter(stateNum, measureNum, 0);
    KF.transitionMatrix = (Mat_<float>(4, 4) <<1,0,1,0,0,1,0,1,0,0,1,0,0,0,0,1);  //转移矩阵A
    setIdentity(KF.measurementMatrix);                                             //测量矩阵H
    setIdentity(KF.processNoiseCov, Scalar::all(1e-5));                            //系统噪声方差矩阵Q
    setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1));                        //测量噪声方差矩阵R
    setIdentity(KF.errorCovPost, Scalar::all(1));                                  //后验错误估计协方差矩阵P
    rng.fill(KF.statePost,RNG::UNIFORM,0,winHeight>winWidth?winWidth:winHeight);   //初始状态值x(0)
    measurement = Mat::zeros(measureNum, 1, CV_32F);                           //初始测量值x'(0)，因为后面要更新这个值，所以必须先定义
}

kalman_filter::~kalman_filter(){}
void kalman_filter::update(Point pt)
{
    //2.kalman prediction
    Mat prediction = KF.predict();
    predict_pt = Point(prediction.at<float>(0),prediction.at<float>(1) );   //预测值(x',y')

    //3.update measurement
    measure_pt.x=pt.x;
    measure_pt.y=pt.y;
    measurement.at<float>(0) = (float)measure_pt.x;
    measurement.at<float>(1) = (float)measure_pt.y;

    //4.update
    KF.correct(measurement);
}

void kalman_filter::draw(Mat &image)
{
    //draw
Mat tpp;
image.copyTo(tpp);
    circle(tpp,predict_pt,5,Scalar(0,255,0),3);    //predicted point with green
    circle(tpp,measure_pt,5,Scalar(255,0,0),3); //current position with red

    char buf[256];
//    printf(buf,256,"predicted position:(%3d,%3d)",predict_pt.x,predict_pt.y);
//    putText(tpp,buf,Point(10,30),FONT_HERSHEY_SCRIPT_COMPLEX,1,Scalar(0,0,0),1,8);
//    printf(buf,256,"current position :(%3d,%3d)",measure_pt.x,measure_pt.y);
//    putText(tpp,buf,Point(10,60),FONT_HERSHEY_SCRIPT_COMPLEX,1,Scalar(0,0,0),1,8);

    imshow("kalman", tpp);
    int key=waitKey(3);

}
