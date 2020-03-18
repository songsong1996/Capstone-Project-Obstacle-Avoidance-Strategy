//
// Created by songsong on 19-3-5.
//

// C++ 标准库

#include <string>
#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <string.h>
#include <dirent.h>
#include <unistd.h>
#include <vector>
using namespace std;


// OpenCV 库
#include <opencv2/core/core.hpp>
#include<opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;

Mat result_color=Mat::zeros(480,640,CV_8UC1);
Mat result;


Mat convertTo3Channels( Mat& binImg)
{
    Mat three_channel = Mat::zeros(binImg.rows, binImg.cols, CV_8UC3);
    vector<Mat> channels;
    for (int i = 0; i < 3; i++) { channels.push_back(binImg); }
    merge(channels, three_channel);
    return three_channel;
}


void change()
{
    for(int i=0;i<480;i++)
    {
        for (int j = 0; j < 640; j++)
        {
            result_color.at<uchar>(i, j) = 255;

        }
    }
    result=convertTo3Channels(result_color);
    cv::circle(result,Point(240,320),30,CV_RGB(0,255,0),2);
}

void draw(Mat &image,vector<Point> &points,string windowname)
{
    Scalar color=Scalar(255,255,0);
    Mat tp;
    tp=convertTo3Channels(image);
    for(int i=0;i<points.size()-1;i++)
    {
        // circle(image,Point(20,20),10,color,2);
        line(tp,points[i],points[i+1],color,3);
    }
    namedWindow(windowname,WINDOW_NORMAL);
    cv::imshow(windowname,tp);
    // cv::waitKey(0);
}


int readFileList(string &basePath,vector<string> &file_list)
{
    DIR *dir;
    struct dirent *ptr;
    char base[1000];


    if ((dir=opendir(basePath.c_str())) == NULL)
    {
        perror("Open dir error...");
        exit(1);
    }

    while ((ptr=readdir(dir)) != NULL)
    {
        if(strcmp(ptr->d_name,".")==0 || strcmp(ptr->d_name,"..")==0)    ///current dir OR parrent dir
            continue;
        else if(ptr->d_type == 8)    ///file
            file_list.emplace_back(basePath+ptr->d_name);
    }
    closedir(dir);

    for(int i=0;i<file_list.size();i++)
        for(int j=0;j<file_list.size()-1;j++)
            if(file_list[j]>file_list[j+1])
            {
                string tp=file_list[j];
                file_list[j]=file_list[j+1];
                file_list[j+1]=tp;
            }
    return 1;
}


Mat stressDepth(Mat &image) {
    Mat result = Mat::zeros(image.rows, image.cols, CV_16UC1);
    cout<<image.at<uint16_t>(100,100)<<endl;
    for (int i = 0; i < image.rows; i++)
        for (int j = 0; j < image.cols; j++)
        {

            if(image.at<uint16_t>(i,j)>40000)result.at<uint16_t >(i,j)=0;
            else
                result.at<uint16_t>(i, j) = image.at<uint16_t>(i, j);
        }
    return result;
}

// //主函数
//int main( int argc, char** argv )
//{
//    namedWindow("tp",WINDOW_NORMAL);
//    //matchshape
//    Mat image_tpp=imread("../data/0.png",IMREAD_UNCHANGED);
//
//    Mat image_tppp=stressDepth(image_tpp);
//    cv::imshow("tp",image_tppp);
//    cv::waitKey(0);
////    equalizeHist( image_tppp, image_tpp );
////    cv::imshow("tp",image_tpp);
////    cv::waitKey(0);
//
//    Mat image=Mat::zeros(image_tppp.rows,image_tppp.cols,CV_8UC1);
//    image_tppp.convertTo(image,CV_8UC1,255.0/65535.0);
//    // equalizeHist( image, image );
//    cv::imshow("tp",image);
//    cv::waitKey(0);
//
//    Mat image_tp=convertTo3Channels(image);
////    cv:imshow("tp",image);
////    cv::waitKey(0);
//    vector<vector<Point>> contours;
//    vector<Vec4i> hierarchy;
//    findContours(image, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
//
//    cout<<"finish"<<endl;
//
//    double max_area=0,avg=0;
//    for(int i=0;i<contours.size();i++)
//    {
//        avg+=contourArea(contours[i]);
//        if(contourArea(contours[i])>max_area)
//            max_area=contourArea(contours[i]);
//    }
//    avg=avg/(1.0*contours.size());
//    for(int i=0;i<contours.size();i++)
//    {
//        Mat tp;
//        image_tp.copyTo(tp);
//        //  draw(tp,contours[i],"tp");
//        if(contourArea(contours[i])>max_area/3.0)
//            drawContours(image_tp,contours,i,Scalar(rand()%255,rand()%255,rand()%255),3);
//    }
//
//    cv::imshow("tp",image_tp);
//    cv::waitKey(0);
////    for(int i=0;i<contours.size()-1;i++)
////    {
////        Mat tp;
////
////        image_tp.copyTo(tp);
////      //  draw(tp,contours[i],"tp");
////        //drawContours(image_tp,contours,i,Scalar(rand()%255,rand()%255,rand()%255));
////        for(int j=i;j<contours.size();j++)
////        {
////            double match=matchShapes(contours[i],contours[j], CONTOURS_MATCH_I2, 1);
////            cout<<to_string(i)<<" & "<<to_string(j)<<"  "<<to_string(match)<<endl;
////           // draw(tp,contours[j],"tp");
////
//////            namedWindow("tp",WINDOW_NORMAL);
//////            cv::imshow("tp",image_tp);
//////            cv::waitKey(0);
////           // drawContours(image_tp,contours,j,Scalar(rand()%255,rand()%255,rand()%255));
////        }
////
////    }
//
//    return 0;
//}

//
//void cvConvertImage2pgm(char* filename,IplImage* srcImage,int type)
//{
//    int width=srcImage->width;
//    int height=srcImage->height;
//
//    FILE *pgmPict;
//    int rSize=width*height;
//    int i,j;
//    pgmPict=fopen(filename,"w");
//    if(type==2)
//    {
//        fprintf(pgmPict,"P2\n");
//    }else if(type==5)
//    {
//        fprintf(pgmPict,"P5\n");
//    }
//    fprintf(pgmPict,"%d %d \n%d\n",width,height,255);
//    if(type==5)
//    {
//        unsigned char temp=0;
//        for( i=0;i<srcImage->height;i++)
//        {
//            for( j=0;j<srcImage->width;j++)
//            {
//                temp=srcImage->imageData[i*srcImage->widthStep+j*3];
//                fwrite((void*)&temp,sizeof(unsigned char),1,pgmPict);
//            }
//        }
//    }
//    else if(type==2)
//    {
//        for( i=0;i<srcImage->height;i++)
//        {
//            for( j=0;j<srcImage->width;j++)
//            {
//                int temp=(int)srcImage->imageData[i*srcImage->widthStep+j*3];
//                if(temp<0)
//                    temp+=256;
//                fprintf(pgmPict,"%d ",temp);
//            }
//        }
//    }
//    fclose(pgmPict);
//}

int main( int argc, char** argv )
{
    Mat tpp=Mat::zeros(500,500,CV_8UC1);
    Rect r;
    r.x=10;
    r.y=100;
    r.width=100;
    r.height=10;
    cv::rectangle(tpp,r.tl(),r.br(), Scalar(200),3);
 //   cv::line(tpp,Point(0,0),Point(250,0),Scalar(255));
//    for(int i=0;i<250;i++)
//        tpp.at<uchar>(i,10)=100;
    imshow("tp",tpp);
    waitKey(0);
//
//Mat tp=imread("./0926_left.png",IMREAD_UNCHANGED);
//cv::imwrite("./0926_left.pmg",tp);



}