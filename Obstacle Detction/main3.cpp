//
// Created by songsong on 19-5-17.
//
#include <iostream>
#include"Hash.h"

int main(int argc, char **argv)
{
//    Mat tp1=imread("../data/33.png",IMREAD_UNCHANGED);
//    Mat tp2=imread("../data/27.png");
//    tp1.convertTo(tp1,CV_8UC3);
//    tp2.convertTo(tp2,CV_8UC3);
//    int d1=HashMatch(tp1,tp2,1);
//    int d2=HashMatch(tp1,tp2,2);
//    int d3=HashMatch(tp1,tp2,3);
//    cout<<d1<<endl<<d2<<endl<<d3<<endl;


//    Mat im3_tp=imread("../data/72.png",IMREAD_UNCHANGED);
//    Mat im3;
//    cvtColor(im3_tp,im3,COLOR_BGR2GRAY);
//        vector<vector<Point>> contours;
//        vector<Vec4i> hierarchy;
//    findContours(im3,contours,hierarchy,RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
//    namedWindow("tp",WINDOW_NORMAL);
//    imshow("tp",im3);
//    cv::waitKey(0);
//
//    im3.copyTo(im3_tp);
//    for(int i=0;i<contours.size();i++)
//    {
//        drawContours(im3_tp,contours,i,Scalar(rand()%255,rand()%255,rand()%255));
//    }
//
//    namedWindow("tp",WINDOW_NORMAL);
//    imshow("tp",im3_tp);
//    cv::waitKey(0);

//    Mat im3_tp=imread("../data/001.jpg",IMREAD_UNCHANGED);
//    Mat im3;
//    im3_tp.convertTo(im3,CV_8UC3);
////    im3.copyTo(im3_tp);
//    cvtColor(im3,im3_tp,COLOR_BGR2GRAY);
//    vector<vector<Point>> contours;
//    vector<Vec4i> hierarchy;
//    findContours(im3_tp,contours,hierarchy,RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
//    namedWindow("tp",WINDOW_NORMAL);
//    imshow("tp",im3_tp);
//    cv::waitKey(0);
//
//    im3.copyTo(im3_tp);
//    for(int i=0;i<contours.size();i++)
//    {
//        drawContours(im3_tp,contours,i,Scalar(rand()%255,rand()%255,rand()%255));
//    }
//
//    namedWindow("tp",WINDOW_NORMAL);
//    imshow("tp",im3_tp);
//    cv::waitKey(0);


Mat im1=imread("../data/tp.png",IMREAD_UNCHANGED);
    Mat im2=imread("../data/tp0.png",IMREAD_UNCHANGED);
    Mat im3=imread("../data/tp1.png",IMREAD_UNCHANGED);

    int d1=HashMatch(im1,im2,2);
    int d2=HashMatch(im1,im3,2);

   cout<<d1<<"  "<<d2<<endl;

   imshow("p1",im1);
   imshow("p2",im2);
    imshow("p3",im3);
    waitKey(0);

    return 0;
}