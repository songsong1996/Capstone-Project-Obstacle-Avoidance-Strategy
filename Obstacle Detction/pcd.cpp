//
// Created by songsong on 19-6-3.
//

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <dirent.h>
#include <unistd.h>
#include<iostream>
#include <vector>
#include<fstream>

#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
using namespace std;
using namespace cv;
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


void SplitString(const string& s, vector<string>& v, const string& c)
{
    string::size_type pos1, pos2;
    pos2 = s.find(c);
    pos1 = 0;
    while(string::npos != pos2)
    {
        v.push_back(s.substr(pos1, pos2-pos1));

        pos1 = pos2 + c.size();
        pos2 = s.find(c, pos1);
    }
    if(pos1 != s.length())
        v.push_back(s.substr(pos1));
}


int main( int argc, char** argv )
{
    string basePath="../data/pcd/txt/";
    vector<string> file_list1;
    readFileList(basePath,file_list1);

    vector<string> file_list2;
    basePath="../data/pcd/color/";
    readFileList(basePath,file_list2);

    for(int i=0;i<file_list1.size();i++)
    {
        //Mat top_view=Mat::zeros(2860,1242,CV_8UC1);
        Mat top_view=Mat::zeros(301,261,CV_8UC1);
        Mat depth=Mat::zeros(375,1242,CV_16UC1);
        Mat depth_tpp=Mat::zeros(375,1242,CV_8UC1);
        Mat color=Mat::zeros(375,1242,CV_8UC3);
        ifstream infile;
        infile.open(file_list1[i].data());   //将文件流对象与文件连接起来
        assert(infile.is_open());   //若失败,则输出错误消息,并终止程序运行

        string s;

        Mat color_tp=imread(file_list2[i],IMREAD_UNCHANGED);
        while(getline(infile,s))
        {
            //cout<<s<<endl;
            vector<string> v;
            SplitString(s, v," "); //可按多个字符来分隔;
            if(atof(v[0].c_str())< 15.0  &&  atoi(v[3].c_str())<1242) {
                if(   int(130 - (20.0 * atof(v[1].c_str())))<225   )
                {
                    top_view.at<uchar>(int(301 - (20.0 * atof(v[0].c_str()))),
                                       int(130 - (20.0 * atof(v[1].c_str())))) = 255;
                    //     top_view.at<uchar>(int(600 - (20.0 * atof(v[0].c_str()))), (int)(atof(v[3].c_str())/4.75) )= 255;
                    depth.at<ushort>(atoi(v[4].c_str()), atoi(v[3].c_str())) = int(401 - (20.0 * atof(v[0].c_str())))*200 ;
                    depth_tpp.at<uchar>(atoi(v[4].c_str()), atoi(v[3].c_str())) = 255;
                    //color.at<Vec3b>(  atoi(v[4].c_str()), atoi(v[3].c_str())   )=  Vec3b(  atoi(v[5].c_str()),atoi(v[6].c_str()),atoi(v[7].c_str()) );
                }
            }
        }
        infile.close();             //关闭文件输入流
         Mat element1 = getStructuringElement(MORPH_RECT, Size(20,20)); //第一个参数MORPH_RECT表示矩形的卷积核，当然还可以选择椭圆形的、交叉型的
        string str=file_list1[i].substr(file_list1[i].find_last_of("/")+1,11)   ;
         imwrite("../data/pcd/dd/d"+str+".png",depth_tpp);
         imwrite("../data/pcd/dd/t"+str+".png",top_view);
    //膨胀操作
    Mat depth_tp;
    dilate(depth_tpp, depth_tp, element1);

        dilate(depth, depth, element1);

        vector<vector<Point>> contours;
        vector<Vec4i> hierarchy;
        findContours(depth_tp, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        for(int i=0;i<color.rows;i++)
            for(int j=0;j<color.cols;j++)
            {
                for(int k=0;k<contours.size();k++)
                {
                   if(  pointPolygonTest(contours[k], Point2f(j, i), false) ==1 )
                   {
                       color.at<Vec3b>(i,j)=color_tp.at<Vec3b>(i,j);
                   }
                }
            }


        //cv::namedWindow("tp",WINDOW_NORMAL);
//        imshow("tp",top_view);
//        imshow("color",color);
        //string str=file_list1[i].substr(file_list1[i].find_last_of("/")+1,11)   ;
        imwrite("../data/pcd/top_view/"+ str+".png",top_view);
        imwrite("../data/pcd/depth/"+ str+".png",depth);
        imwrite("../data/pcd/color2/"+ str+".png",color);
        cout<<str<<endl;
//        imshow("depth",depth_tp);
//       imshow("depth2",depth);
//        waitKey(0);
    }
    return 0;
}


//
//int main( int argc, char** argv )
//{
//    string basePath="../data/pcd/depth/";
//    vector<string> file_list1;
//    readFileList(basePath,file_list1);
//
//    vector<string> file_list2;
//    basePath="../data/pcd/color/";
//    readFileList(basePath,file_list2);
//
//    for(int i=0;i<file_list1.size();i++)
//    {
//        //Mat top_view=Mat::zeros(2860,1242,CV_8UC1);
//        Mat top_view=Mat::zeros(301,261,CV_8UC1);
//        Mat depth=imread(file_list1[i],IMREAD_UNCHANGED);
//        Mat depth_tpp=Mat::zeros(375,1242,CV_8UC1);
//        Mat color=Mat::zeros(375,1242,CV_8UC3);
//        ifstream infile;
//        infile.open(file_list1[i].data());   //将文件流对象与文件连接起来
//        assert(infile.is_open());   //若失败,则输出错误消息,并终止程序运行
//
//        string s;
//
//        Mat color_tp=imread(file_list2[i],IMREAD_UNCHANGED);
//        for(int ii=0;ii<depth.rows;ii++)
//            for(int jj=0;jj<depth.cols;jj++)
//            {
//                if()
//            }
//
//
//
//
//        while(getline(infile,s))
//        {
//            //cout<<s<<endl;
//            vector<string> v;
//            SplitString(s, v," "); //可按多个字符来分隔;
//            if(atof(v[0].c_str())< 15.0   &&  atoi(v[3].c_str())<1242) {
//                top_view.at<uchar>(int(301 - (20.0 * atof(v[0].c_str()))),
//                                   int(130 - (20.0 * atof(v[1].c_str())))) = 255;
//                //     top_view.at<uchar>(int(600 - (20.0 * atof(v[0].c_str()))), (int)(atof(v[3].c_str())/4.75) )= 255;
//                depth.at<ushort>(atoi(v[4].c_str()), atoi(v[3].c_str())) = int(301 - (20.0 * atof(v[0].c_str())))*200 ;
//                depth_tpp.at<uchar>(atoi(v[4].c_str()), atoi(v[3].c_str())) = 255;
//                //color.at<Vec3b>(  atoi(v[4].c_str()), atoi(v[3].c_str())   )=  Vec3b(  atoi(v[5].c_str()),atoi(v[6].c_str()),atoi(v[7].c_str()) );
//            }
//        }
//        infile.close();             //关闭文件输入流
//        Mat element1 = getStructuringElement(MORPH_RECT, Size(20,20)); //第一个参数MORPH_RECT表示矩形的卷积核，当然还可以选择椭圆形的、交叉型的
//        //膨胀操作
//        Mat depth_tp;
//        dilate(depth_tpp, depth_tp, element1);
//
//        dilate(depth, depth, element1);
//
//        vector<vector<Point>> contours;
//        vector<Vec4i> hierarchy;
//        findContours(depth_tp, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
//
//        for(int i=0;i<color.rows;i++)
//            for(int j=0;j<color.cols;j++)
//            {
//                for(int k=0;k<contours.size();k++)
//                {
//                    if(  pointPolygonTest(contours[k], Point2f(j, i), false) ==1 )
//                    {
//                        color.at<Vec3b>(i,j)=color_tp.at<Vec3b>(i,j);
//                    }
//                }
//            }
//
//
//        //cv::namedWindow("tp",WINDOW_NORMAL);
////        imshow("tp",top_view);
////        imshow("color",color);
//        string str=file_list1[i].substr(file_list1[i].find_last_of("/")+1,11)   ;
//        imwrite("../data/pcd/top_view/"+ str+".png",top_view);
//        imwrite("../data/pcd/depth/"+ str+".png",depth);
//        imwrite("../data/pcd/color2/"+ str+".png",color);
//        cout<<str<<endl;
////        imshow("depth",depth_tp);
////       imshow("depth2",depth);
////        waitKey(0);
//    }
//    return 0;
//}
//



