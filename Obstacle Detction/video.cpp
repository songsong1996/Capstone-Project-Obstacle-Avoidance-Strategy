//
// Created by songsong on 19-6-11.
//

#include <iostream>
#include <string>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <dirent.h>
#include <unistd.h>
#include<iostream>
#include <vector>
#include<fstream>

using namespace std;
using namespace cv;


void  Image_To_Video();
int readFileList(string &basePath,vector<string> &file_list);
int main()
{
    string video_name = "test.avi";//注意，使用string时，若不用using namespace std，需要使用std::string
    Image_To_Video();
    return 0;
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



void Image_To_Video()
{
    vector<string> color_raw,depth_raw,color_draw,depth_2,flow,top_view;
    string s1="../data/pcd/color/",s2="../data/pcd/depth-now/",s3="../data/pcd/color_result/",s4="../data/pcd/depth2/",s5="../data/pcd/flow/",s6="../data/pcd/result/";
    int ii=readFileList(s1,color_raw);
    ii=readFileList(s2,depth_raw);
    ii=readFileList(s3,color_draw);
    ii=readFileList(s4,depth_2);
    ii=readFileList(s5,flow);
    ii=readFileList(s6,top_view);


    char image_name[20];
    string s_image_name;
    cv::VideoWriter writer;
    int isColor = 1;//不知道是干啥用的

    int frame_fps = 10;
    int frame_width = 563;
    int frame_height = 1242;
    using namespace cv;
    string video_name = "../data/out.avi";
    writer = VideoWriter(video_name, VideoWriter::fourcc('M', 'J', 'P', 'G'),frame_fps,Size(frame_width,frame_height),isColor);
    cout << "frame_width is " << frame_width << endl;
    cout << "frame_height is " << frame_height << endl;
    cout << "frame_fps is " << frame_fps << endl;
    cv::namedWindow("image to video", WINDOW_AUTOSIZE);
    int num = 66;//输入的图片总张数
    int i = 0;
    Mat img;



    for(int i=0;i<color_draw.size();i++)
    {

        Mat sum=Mat::ones(1125,2484,CV_8UC3);
        Mat color=imread(color_raw[i],IMREAD_UNCHANGED);
        Mat depth=imread(depth_raw[i],IMREAD_UNCHANGED);
        Mat colordraw=imread(color_draw[i],IMREAD_UNCHANGED);
        Mat depth2=imread(depth_2[i],IMREAD_UNCHANGED);
        Mat fflow=imread(flow[i],IMREAD_UNCHANGED);
        Mat topview=imread(top_view[i],IMREAD_UNCHANGED);
        Mat ttop=Mat::ones(375,1242,CV_8UC3);
       // int w1=(int)(261.0/300.0*375.0);
        int pp1=(1242-261)/2,pp2=(375-300)/2;
       // topview.resize(375,(int)(261.0/300.0*375.0));
        for(int k=pp2;k<300+pp2;k++)
            for(int j=pp1;j<261+pp1;j++)
            {
                for(int kk=0;kk<3;kk++)
                    ttop.at<Vec3b>(k,j)[kk]=topview.at<Vec3b>(k-pp2,j-pp1)[kk];
            }
//        imshow("pp",ttop);
//            waitKey(0);

        for(int k=0;k<=374;k++)
            for(int j=0;j<=1241;j++)
            {
                for(int kk=0;kk<3;kk++)
                    sum.at<Vec3b>(k,j)[kk]=color.at<Vec3b>(k,j)[kk];
            }
        for(int k=0;k<=374;k++)
            for(int j=1242;j<=2483;j++)
            {
                for(int kk=0;kk<3;kk++)
                    sum.at<Vec3b>(k,j)[kk]=depth.at<ushort>(k,j-1242)*255.0/65535.0;
            }

        for(int k=375;k<=749;k++)
            for(int j=0;j<=1241;j++)
            {
                for(int kk=0;kk<3;kk++)
                    sum.at<Vec3b>(k,j)[kk]=colordraw.at<Vec3b>(k-375,j)[kk];
            }
        for(int k=375;k<=749;k++)
            for(int j=1242;j<=2483;j++)
            {
                for(int kk=0;kk<3;kk++)
                    sum.at<Vec3b>(k,j)[kk]=depth2.at<ushort>(k-375,j-1242)*255.0/65535.0;
            }



        for(int k=750;k<=1124;k++)
            for(int j=0;j<=1241;j++)
            {
                for(int kk=0;kk<3;kk++)
                    sum.at<Vec3b>(k,j)[kk]=fflow.at<Vec3b>(k-750,j)[kk];
            }
        for(int k=750;k<=1124;k++)
            for(int j=1242;j<=2483;j++)
            {
                for(int kk=0;kk<3;kk++)
                    sum.at<Vec3b>(k,j)[kk]=ttop.at<Vec3b>(k-750,j-1242)[kk];
            }
//        imshow("sum",sum);
//            waitKey(0);



            //写入
            writer.write(sum);
        stringstream ss;
        int a = i;
        ss<<a;
        string b = ss.str();

            imwrite("../data/sum/"+b+".png",sum);
            if (cv::waitKey(30) == 27 || i == 160)
            {
                cout << "按下ESC键" << endl;
                break;
            }

    }

writer.release();
    cout << "---------------Video_To_Image-----------------" << endl;


}


