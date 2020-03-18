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

//void writein(string filename,string writein)
//{
//
//    ofstream stream(filename);
//    stream<<writein;
//
//}

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

//// 主函数
//int main( int argc, char** argv )
//{
////    ofstream stream_false("../txt_false.txt");
////    ofstream stream_true("../txt_true.txt");
//   ofstream stream_all("../txt_all_tp.txt");
//
//
//    string filepath1="../data/";
//
//    vector<string> file_list;
//    file_list.empty();
//    int tp;
//    tp=readFileList(filepath1,file_list);
//
//    for(int k=0;k<file_list.size()-1;k++)
//    {
//        Mat image1,image2;
//        image1= imread(file_list.at(k),IMREAD_UNCHANGED );
//        image2=imread(file_list.at(k+1),IMREAD_UNCHANGED );
//
//        vector<vector<Point>> contours1,contours2;
//        vector<Vec4i> hierarchy1,hierarchy2;
//        findContours(image1, contours1, hierarchy1, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
//        findContours(image2, contours2, hierarchy2, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
//
//
//        for(int i=0;i<contours1.size();i++)
//        {
//            Mat tp;
//            image1.copyTo(tp);
//            draw(tp,contours1[i],"1");
//            int num=0;
//            double min=100;
//            double area_t;
//            bool is_exist=false;
//            for(int j=0;j<contours2.size();j++)
//            {
//                Mat tpp;
//                image2.copyTo(tpp);
//                double match=matchShapes(contours1[i],contours2[j], CONTOURS_MATCH_I2, 1);
//                double area=contourArea(contours1[i])>contourArea(contours2[j])?
//                            contourArea(contours2[j])/contourArea(contours1[i]):contourArea(contours1[i])/contourArea(contours2[j]);
//
//                stream_all<<to_string(k)<<"  "<<to_string(i)<<"  "<<to_string(j)<<"  "<<to_string(match)<<"  "<<to_string(area)<<"  "<<to_string(match/(area*area))<<endl;
//
//            }
//
//        }
//        cout<<endl;
//    }
//
//    return 0;
//}




// 主函数
int main( int argc, char** argv )
{
//    ofstream stream_false("../txt_false.txt");
//    ofstream stream_true("../txt_true.txt");
//    ofstream stream_all("../txt_all.txt");
    // FILE * fid = fopen("../data/txt_out.txt","w");


//    string filepath1="../data/";
//
//    vector<string> file_list;
//    file_list.empty();
//    int tp;
//    tp=readFileList(filepath1,file_list);
//
//    for(int k=0;k<file_list.size()-1;k++)
//    {
//        Mat image1,image2;
//        image1= imread(file_list.at(k),IMREAD_UNCHANGED );
//        image2=imread(file_list.at(k+1),IMREAD_UNCHANGED );
//
//        vector<vector<Point>> contours1,contours2;
//        vector<Vec4i> hierarchy1,hierarchy2;
//        findContours(image1, contours1, hierarchy1, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
//        findContours(image2, contours2, hierarchy2, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
//
//
//        for(int i=0;i<contours1.size();i++)
//        {
//            Mat tp;
//            image1.copyTo(tp);
//            draw(tp,contours1[i],"1");
//            int num=0;
//            double min=100;
//            double area_t;
//            bool is_exist=false;
//            for(int j=0;j<contours2.size();j++)
//            {
//
//                double match=matchShapes(contours1[i],contours2[j], CONTOURS_MATCH_I2, 1);
//                double area=contourArea(contours1[i])>contourArea(contours2[j])?
//                            contourArea(contours2[j])/contourArea(contours1[i]):contourArea(contours1[i])/contourArea(contours2[j]);
//                if((match>5 or area<0.3 or match/(area*area)>5)==false)
//                {
//                    is_exist = true;
//
//                    if(match<min) {
//                        cout << to_string(k) << "  " << to_string(i) << "  " << to_string(j) << "  " << to_string(match)
//                             << "  " << to_string(area) <<"  "<<to_string(match/(area*area))<< endl;
//                        num = j;
//                        area_t = area;
//                        min = match;
//
//                    }
//                }
////                if (match<min)
////                {
////                    num=j;
////                    area_t=area;
////                    min=match;
////                }
////                cout<<to_string(i)<<" & "<<to_string(j)<<"  "<<to_string(match)<<"  "<<to_string(area)<<endl;
//
//
//
//
////            namedWindow("tp",WINDOW_NORMAL);
////            cv::imshow("tp",image_tp);
//
//                // drawContours(image_tp,contours,j,Scalar(rand()%255,rand()%255,rand()%255));
//            }
//            if(is_exist) {
//                Mat tpp;
//                image2.copyTo(tpp);
//                draw(tpp, contours2[num], "2");
//                cv::waitKey(0);
//            }
//
////            if(is_exist)
////            {
////                Mat tpp;
////                image2.copyTo(tpp);
////                draw(tpp, contours2[num], "3");
////                cout << to_string(i) << " & " << to_string(num) << "  " << to_string(min) << "  " << to_string(area_t)
////                     << endl;
////
////                if (waitKey(0) == 90) {
////                    stream_false << to_string(k) << " & " << to_string(i) << " & " << to_string(num) << "  "
////                                 << to_string(min) << "  " << to_string(area_t) << endl;
////                } else {
////                    stream_true << to_string(k) << " & " << to_string(i) << " & " << to_string(num) << "  "
////                                << to_string(min) << "  " << to_string(area_t) << endl;
////                    stream_all << to_string(k) << " & " << to_string(i) << " & " << to_string(num) << "  "
////                               << to_string(min) << "  " << to_string(area_t) << endl;
////                }
////            }
//        }
//        cout<<endl;
//    }


//        for(int i=0;i<contours1.size();i++)
//        {
//            Mat tp;
//            image1.copyTo(tp);
//            draw(tp,contours1[i],"1");
//            //drawContours(image_tp,contours,i,Scalar(rand()%255,rand()%255,rand()%255));
//            int num=0;
//            double min=100;
//            double area_t;
//            for(int j=0;j<contours2.size();j++)
//            {
//                double match=matchShapes(contours1[i],contours2[j], CONTOURS_MATCH_I2, 1);
//                double area=contourArea(contours1[i])>contourArea(contours2[j])?
//                            contourArea(contours2[j])/contourArea(contours1[i]):contourArea(contours1[i])/contourArea(contours2[j]);
//                if((match>5 or area<0.3 or match/(area*area)>5)==false)
//                {
//                    num=j;
//                    area_t=area;
//                    min=match;
//                }
////                if (match<min)
////                {
////                    num=j;
////                    area_t=area;
////                    min=match;
////                }
////                cout<<to_string(i)<<" & "<<to_string(j)<<"  "<<to_string(match)<<"  "<<to_string(area)<<endl;
////                draw(tpp,contours2[j],"2");
//
//
//
////            namedWindow("tp",WINDOW_NORMAL);
////            cv::imshow("tp",image_tp);
////            cv::waitKey(0);
//                // drawContours(image_tp,contours,j,Scalar(rand()%255,rand()%255,rand()%255));
//            }
//            Mat tpp;
//            image2.copyTo(tpp);
//            draw(tpp,contours2[num],"2");
//            cout<<to_string(i)<<" & "<<to_string(num)<<"  "<<to_string(min)<<"  "<<to_string(area_t)<<endl;
//
//            if(waitKey(0)==90)
//            {
//                stream_false<<to_string(k)<<" & "<<to_string(i)<<" & "<<to_string(num)<<"  "<<to_string(min)<<"  "<<to_string(area_t)<<endl;
//            }
//            else
//            {
//                stream_true<<to_string(k)<<" & "<<to_string(i)<<" & "<<to_string(num)<<"  "<<to_string(min)<<"  "<<to_string(area_t)<<endl;
//                stream_all<<to_string(k)<<" & "<<to_string(i)<<" & "<<to_string(num)<<"  "<<to_string(min)<<"  "<<to_string(area_t)<<endl;
//
//            }
//            //cv::waitKey(0);
//        }
//        cout<<endl;
//    }

//    stream_false.close();
//    stream_true.close();
//    stream_all.close();

    //matchshape
    Mat image_tpp=imread("../data/numbers.png",IMREAD_GRAYSCALE );
    Mat image=Mat::zeros(image_tpp.rows,image_tpp.cols,CV_8UC1);
    image_tpp.convertTo(image,CV_8UC1);
    Mat image_tp=convertTo3Channels(image);
//    cv:imshow("tp",image);
//    cv::waitKey(0);
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(image, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    cout<<"finish"<<endl;

    for(int i=0;i<contours.size()-1;i++)
    {
        Mat tp;

        image_tp.copyTo(tp);
        draw(tp,contours[i],"tp");
        //drawContours(image_tp,contours,i,Scalar(rand()%255,rand()%255,rand()%255));
        for(int j=i+1;j<contours.size();j++)
        {
            double match=matchShapes(contours[i],contours[j], CONTOURS_MATCH_I2, 1);
            cout<<to_string(i)<<" & "<<to_string(j)<<"  "<<to_string(match)<<endl;
            draw(tp,contours[j],"tp");

//            namedWindow("tp",WINDOW_NORMAL);
//            cv::imshow("tp",image_tp);
//            cv::waitKey(0);
           // drawContours(image_tp,contours,j,Scalar(rand()%255,rand()%255,rand()%255));
        }

    }





//    std::vector<int> vec;
//    for(int i=0;i<100;i++)
//    {
//        vec.push_back(i);
//    }
//
//    printf("10:%d\n",vec[10]);
//    printf("size:%d\n",vec.size());
//    printf("**********************************\n");
//    std::vector<int>::iterator it = vec.begin()+10;
//    vec.erase(it);
//
//    printf("10:%d\n",vec[10]);
//    printf("size:%d\n",vec.size());
//    imshow("tpp",result_color);
//    cv::waitKey(0);
//    change();
//    imshow("tp",result);
//    cv::waitKey(0);

    return 0;
}



//int main( int argc, char** argv )
//{
//    vector<int> v1,v2,v3;
//    v1.push_back(0);
//    v1.push_back(1);
//
//    v2.push_back(5);
//    v2.push_back(4); v2.push_back(3);
//
//
//    v3.push_back(0);
//    v3.push_back(1);
//
//    v1.clear();
//    cout<<v1.size()<<" "<<v1[0]<<" "<<v1[1]<<endl;
//    v1.assign(v2.begin(),v2.end());
//    cout<<v1.size()<<" "<<v1[0]<<" "<<v1[1]<<endl;
//
//
//    v2.empty();
//    cout<<v2.size()<<" "<<v2[0]<<" "<<v2[1]<<endl;
//
//////    std::vector<int>temp(v3);//1
//////    temp.swap(v3);//2
////    vector <int>().swap(v3);
////    cout<<v3.size()<<" "<<v3[0]<<" "<<v3[1]<<endl;
//
//
//
//    return 0;
//}


