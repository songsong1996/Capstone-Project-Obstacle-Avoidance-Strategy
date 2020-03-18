
#include"kalman/kalman.h"
#include "opencv2/video/tracking.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdio.h>
#include <iostream>


using namespace cv;
using namespace std;

Mat convertTo3Channels( Mat &binImg)
{
    Mat three_channel = Mat::zeros(binImg.rows, binImg.cols, CV_8UC3);
    vector<Mat> channels;
    for (int i = 0; i < 3; i++)
    { channels.push_back(binImg); }
    merge(channels, three_channel);
    return three_channel;
}

kalman_filter::kalman_filter()
{}


kalman_filter::kalman_filter(Point2f pt)
{
    int winHeight=600;
    int winWidth=640;
    RNG rng;
    //1.kalman filter setup
    const int stateNum=6;                                      //状态值4×1向量(x,y,△x,△y)
    const int measureNum=2;                                    //测量值2×1向量(x,y)
    KF=KalmanFilter(stateNum, measureNum, 0);
    KF.transitionMatrix = (Mat_<float>(6,6)
            <<1,0,1,0,1,0,
            0,1,0,1,0,1,
            0,0,1,0,1,0,
            0,0,0,1,0,1,
            0,0,0,0,1,0,
            0,0,0,0,0,1);  //转移矩阵A
    setIdentity(KF.measurementMatrix);                                             //测量矩阵H
    setIdentity(KF.processNoiseCov, Scalar::all(1e-5));                            //系统噪声方差矩阵Q
    setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1));                        //测量噪声方差矩阵R
    setIdentity(KF.errorCovPost, Scalar::all(1));                                  //后验错误估计协方差矩阵P
    //rng.fill(KF.statePost,RNG::UNIFORM,0,winHeight>winWidth?winWidth:winHeight);   //初始状态值x(0)
    randn(KF.statePost, Scalar::all(0), Scalar::all(0.1));
    measurement = Mat::zeros(measureNum, 1, CV_32F);                           //初始测量值x'(0)，因为后面要更新这个值，所以必须先定义
    measurement.at<float>(0) = (float)pt.x;
    measurement.at<float>(1) = (float)pt.y;

}

kalman_filter::~kalman_filter(){}

void kalman_filter::update(Point2f pt)
{
    //2.kalman prediction
    Mat prediction = KF.predict();
    predict_pt = Point2f(prediction.at<float>(0),prediction.at<float>(1) );   //预测值(x',y')
   // cout<<prediction.at<float>(0)<<" "<<prediction.at<float>(1)<<endl;

    //3.update measurement
    measure_pt.x=pt.x;
    measure_pt.y=pt.y;

    measurement.at<float>(0) = (float)measure_pt.x;
    measurement.at<float>(1) = (float)measure_pt.y;

    //4.update
    KF.correct(measurement);
}

Point2f kalman_filter::get_predict_pt()
{
    Point2f pt;
    pt.x=predict_pt.x;
    pt.y=predict_pt.y;
    return pt;
}
Point2f kalman_filter::predict_kt(int kt)
{
    KalmanFilter kf2(6,2);
    KF.controlMatrix.copyTo(kf2.processNoiseCov);
    KF.errorCovPost.copyTo(kf2.errorCovPost);
    KF.gain.copyTo(kf2.gain);
    KF.measurementMatrix.copyTo(kf2.measurementMatrix);
    KF.statePost.copyTo(kf2.statePost);
    KF.statePre.copyTo(kf2.statePre);
    KF.temp1.copyTo(kf2.temp1);
    KF.temp2.copyTo(kf2.temp2);
    KF.temp3.copyTo(kf2.temp3);
    KF.temp4.copyTo(kf2.temp4);
    KF.temp5.copyTo(kf2.temp5);
    KF.transitionMatrix.copyTo(kf2.transitionMatrix);
    KF.processNoiseCov.copyTo(kf2.processNoiseCov);
    KF.errorCovPre.copyTo(kf2.errorCovPre);
    KF.measurementNoiseCov.copyTo( kf2.measurementNoiseCov);

    //cout<< Pt.x<<" "<<Pt.y<<endl;
    // kf2.correct(measurement);
    Mat prediction2 = kf2.predict();
    //cout<<"1 "<< ((float*)prediction2.data)[0]<<" "<< ((float*)prediction2.data)[1]<<endl;
    for(int i=0;i<kt-1;i++)
    {
        prediction2 = kf2.predict();
       // cout<<"2 "<<  ((float*)prediction2.data)[0]<<" "<< ((float*)prediction2.data)[1]<<endl;
    }
//    prediction2 = kf2.predict();
//    cout<<"3 "<<  ((float*)prediction2.data)[0]<<" "<< ((float*)prediction2.data)[1]<<endl;
//    prediction2 = kf2.predict();
//    cout<<"4 "<<  ((float*)prediction2.data)[0]<<" "<< ((float*)prediction2.data)[1]<<endl;
//    prediction2 = kf2.predict();
//    cout<<"5 "<<  ((float*)prediction2.data)[0]<<" "<< ((float*)prediction2.data)[1]<<endl<<endl;
//            prediction2 = kf2.predict();
//            cout<<"6 "<<  ((float*)prediction2.data)[0]<<" "<< ((float*)prediction2.data)[1]<<endl;
//            prediction2 = kf2.predict();
//            cout<<"7 "<<  ((float*)prediction2.data)[0]<<" "<< ((float*)prediction2.data)[1]<<endl;
//            prediction2 = kf2.predict();
//            cout<<"8 "<<  ((float*)prediction2.data)[0]<<" "<< ((float*)prediction2.data)[1]<<endl<<endl;
    Point2f tppp;
    tppp.x= ((float*)prediction2.data)[0];
    tppp.y= ((float*)prediction2.data)[1];
    return tppp;
}



//void kalman_filter::output(Mat &tp)
//{
//    for(int i=0;i<tp.rows;i++)
//        for(int j=0;j<tp.cols;j++)
//        {
//
//        }
//
//}
void kalman_filter::draw(Mat &image)
{
    //draw
    Mat tpp=convertTo3Channels(image);
    image.setTo(Scalar(255,255,255,0));
    circle(tpp,predict_pt,5,Scalar(0,255,0),3);    //predicted point with green
    circle(tpp,measure_pt,5,Scalar(255,0,0),3); //current position with red

    char buf[256];
    printf(buf,256,"predicted position:(%3d,%3d)",predict_pt.x,predict_pt.y);
    putText(tpp,buf,Point(10,30),FONT_HERSHEY_SCRIPT_COMPLEX,1,Scalar(0,0,0),1,8);
    printf(buf,256,"current position :(%3d,%3d)",measure_pt.x,measure_pt.y);
    putText(tpp,buf,Point(10,60),FONT_HERSHEY_SCRIPT_COMPLEX,1,Scalar(0,0,0),1,8);

    imshow("kalman", tpp);
    int key=waitKey(3);
}
