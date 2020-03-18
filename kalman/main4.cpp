//
// Created by songsong on 19-5-28.
//

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>
#include <iostream>
#include<time.h>


using namespace cv;
using namespace std;
Mat img(1000, 1000, CV_8UC3);
Point Pt,Pt_pre;
Mat state(4, 1, CV_32F);


static void onMouse(int event, int x, int y, int, void*)
{
    if (event == EVENT_MOUSEMOVE)
    {
        Pt_pre = Pt;
        //  Pt = Point(x, y);
        Pt.x=Pt.x+5;
        Pt.y=Pt.y+5;
    }
}

int getTime()
{
    return clock()/CLOCKS_PER_SEC;
}

int main()
{
    //RNG rng;
    Mat measurement = Mat::zeros(2, 1, CV_32F);
    Mat measurement2 = Mat::zeros(2, 1, CV_32F);
    randn(state, Scalar::all(0), Scalar::all(0.1));
    Point kp(0,0);
    state.setTo(0);
    KalmanFilter kf(6,2);
    kf.transitionMatrix = (Mat_<float>(6,6)
            <<1,0,1,0,1,0,
            0,1,0,1,0,1,
            0,0,1,0,1,0,
            0,0,0,1,0,1,
            0,0,0,0,1,0,
            0,0,0,0,0,1);  //转移矩阵A
    setIdentity(kf.measurementMatrix);                                             //测量矩阵H
    setIdentity(kf.processNoiseCov, Scalar::all(1e-5));                            //系统噪声方差矩阵Q
    setIdentity(kf.measurementNoiseCov, Scalar::all(1e-1));                        //测量噪声方差矩阵R
    setIdentity(kf.errorCovPost, Scalar::all(1));                                  //后验错误估计协方差矩阵P


    img.setTo(0);
    namedWindow("white", WINDOW_AUTOSIZE);
    // setMouseCallback("white", onMouse);
    int lastTime = 0;
    Pt.x=0;
    Pt.y=0;
    int kk=0;
    double a=0.2;

    for (;;)
    {


        Mat prediction = kf.predict();


        img.setTo(Scalar(255,255,255));
        measurement = (Mat_<float>(2,1) << Pt.x, Pt.y);


        if(kk>10)
        {
            KalmanFilter kf2(6,2);
            kf.controlMatrix.copyTo(kf2.processNoiseCov);
            kf.errorCovPost.copyTo(kf2.errorCovPost);
            kf.gain.copyTo(kf2.gain);
            kf.measurementMatrix.copyTo(kf2.measurementMatrix);
            kf.statePost.copyTo(kf2.statePost);
            kf.statePre.copyTo(kf2.statePre);
            kf.temp1.copyTo(kf2.temp1);
            kf.temp2.copyTo(kf2.temp2);
            kf.temp3.copyTo(kf2.temp3);
            kf.temp4.copyTo(kf2.temp4);
            kf.temp5.copyTo(kf2.temp5);
            kf.transitionMatrix.copyTo(kf2.transitionMatrix);
            kf.processNoiseCov.copyTo(kf2.processNoiseCov);
            kf.errorCovPre.copyTo(kf2.errorCovPre);
            kf.measurementNoiseCov.copyTo( kf2.measurementNoiseCov);

            cout<< Pt.x<<" "<<Pt.y<<endl;
            // kf2.correct(measurement);
            Mat prediction2 = kf2.predict();
            cout<<"1 "<< ((float*)prediction2.data)[0]<<" "<< ((float*)prediction2.data)[1]<<endl;
            prediction2 = kf2.predict();
            cout<<"2 "<<  ((float*)prediction2.data)[0]<<" "<< ((float*)prediction2.data)[1]<<endl;
            prediction2 = kf2.predict();
            cout<<"3 "<<  ((float*)prediction2.data)[0]<<" "<< ((float*)prediction2.data)[1]<<endl;
            prediction2 = kf2.predict();
            cout<<"4 "<<  ((float*)prediction2.data)[0]<<" "<< ((float*)prediction2.data)[1]<<endl;
            prediction2 = kf2.predict();
            cout<<"5 "<<  ((float*)prediction2.data)[0]<<" "<< ((float*)prediction2.data)[1]<<endl<<endl;
//            prediction2 = kf2.predict();
//            cout<<"6 "<<  ((float*)prediction2.data)[0]<<" "<< ((float*)prediction2.data)[1]<<endl;
//            prediction2 = kf2.predict();
//            cout<<"7 "<<  ((float*)prediction2.data)[0]<<" "<< ((float*)prediction2.data)[1]<<endl;
//            prediction2 = kf2.predict();
//            cout<<"8 "<<  ((float*)prediction2.data)[0]<<" "<< ((float*)prediction2.data)[1]<<endl<<endl;
            //   measurement2 = (Mat_<float>(2,1) << Pt.x, Pt.y);

            circle(img, Point(((float*)prediction2.data)[0], (((float*)prediction2.data)[1])), 6, Scalar(255, 255, 0), -1);
            Point tpp;
//            tpp.x=a*(kk+5.0)*(kk+5.0)/2.0;
//            tpp.y=a*(kk+5.0)*(kk+5.0)/2.0;
            tpp.x=Pt.x+25;
            tpp.y=Pt.y+25;

            circle(img, tpp, 4, Scalar(0, 255, 255), -1);
            //    kf2.correct(measurement2);
        }
        circle(img, Pt, 6, Scalar(255,0,0), -1);
        //circle(img, Point(((float*)prediction.data)[0], (((float*)prediction.data)[1])), 6, Scalar(0, 0, 255), -1);

        imshow("white", img);
        waitKey(500);
        if(waitKey(1) == 27)
            break;
        kf.correct(measurement);
//        Pt.x=a*kk*kk/2.0;
//        Pt.y=a*kk*kk/2.0;
        kk+=1;
        Pt.x+=5;
        Pt.y+=5;
    }


    return 0;
}
