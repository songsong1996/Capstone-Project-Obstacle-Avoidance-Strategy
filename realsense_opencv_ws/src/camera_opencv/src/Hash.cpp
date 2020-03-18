//
// Created by songsong on 19-5-17.
//



#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include "camera_opencv/Hash.h"

using namespace std;
using namespace cv;

//均值Hash算法
string aHashValue(Mat &image)
{
    string rst(64,'\0');
    Mat img;
    if(image.channels()==3)
        cvtColor(image,img,COLOR_BGR2GRAY);
    else
        img=image.clone();
    /*第一步，缩小尺寸。
      将图片缩小到8x8的尺寸，总共64个像素,去除图片的细节*/

    resize(img,img,Size(8,8));
    /* 第二步，简化色彩(Color Reduce)。
       将缩小后的图片，转为64级灰度。*/

    uchar *pData;
    for(int i=0;i<img.rows;i++)
    {
        pData = img.ptr<uchar>(i);
        for(int j=0;j<img.cols;j++)
        {
            pData[j]=pData[j]/4;    		}
    }

    /* 第三步，计算平均值。
    计算所有64个像素的灰度平均值。*/
    int average = mean(img).val[0];

    /* 第四步，比较像素的灰度。
  将每个像素的灰度，与平均值进行比较。大于或等于平均值记为1,小于平均值记为0*/
    Mat mask= (img>=(uchar)average);

    /* 第五步，计算哈希值。*/
    int index = 0;
    for(int i=0;i<mask.rows;i++)
    {
        pData = mask.ptr<uchar>(i);
        for(int j=0;j<mask.cols;j++)
        {
            if(pData[j]==0)
                rst[index++]='0';
            else
                rst[index++]='1';
        }
    }
    return rst;
}


//pHash算法
string pHashValue(Mat &image)
{
    Mat img ,dst;
    string rst(64,'\0');
    double dIdex[64];
    double mean = 0.0;
    int k = 0;
    if(image.channels()==3)
    {
        cvtColor(image,image,COLOR_BGR2GRAY);
        img = Mat_<double>(image);
    }
    else
    {
        img = Mat_<double>(image);
    }

    /* 第一步，缩放尺寸*/
    resize(img, img, Size(32,32));

    /* 第二步，离散余弦变换，DCT系数求取*/
    dct(img, dst);

    /* 第三步，求取DCT系数均值（左上角8*8区块的DCT系数）*/
    for (int i = 0; i < 8; ++i) {
        for (int j = 0; j < 8; ++j)
        {
            dIdex[k] = dst.at<double>(i, j);
            mean += dst.at<double>(i, j)/64;
            ++k;
        }
    }

    /* 第四步，计算哈希值。*/
    for (int i =0;i<64;++i)
    {
        if (dIdex[i]>=mean)
        {
            rst[i]='1';
        }
        else
        {
            rst[i]='0';
        }
    }
    return rst;
}

string dHashValue(Mat &image)
{
    string rst(64,'\0');
    Mat img;
    if(image.channels()==3)
        cvtColor(image,img,COLOR_BGR2GRAY);
    else
        img=image.clone();
    /*第一步，缩小尺寸。
      将图片缩小到8x8的尺寸，总共64个像素,去除图片的细节*/


    resize(img,img,Size(8,9));
//
//    for(int i=0;i<8;i++)
//        for(int j=0;j<8;j++)
//            img.at<uchar>(i,j)=img.at<uchar>(i,j)/4;

            /* 第二步，简化色彩(Color Reduce)。
               将缩小后的图片，转为64级灰度。*/

//        /* 第三步，计算平均值。
// 	   计算所有64个像素的灰度平均值。*/
    Mat dist=Mat::zeros(8,8,CV_8UC1);

    for(int i=0;i<8;i++) //row
        for(int j=0;j<8;j++)  //col
        {
            dist.at<uchar>(i,j)=image.at<uchar>(i,j+1)-image.at<uchar>(i,j);
        }

    /* 第五步，计算哈希值。*/
    int index = 0;
    for(int i=0;i<dist.rows;i++)
        for(int j=0;j<dist.cols;j++)
        {
            if(dist.at<uchar>(i,j)>0)
                rst[index++]='1';
            else
                rst[index++]='0';
        }
    return rst;
}

//汉明距离计算
int HanmingDistance(string &str1,string &str2)
{
    if((str1.size()!=64)||(str2.size()!=64))
        return -1;
    int difference = 0;
    for(int i=0;i<64;i++)
    {
        if(str1[i]!=str2[i])
            difference++;
    }
    return difference;
}

int HashMatch(Mat &image1,Mat &image2,int type)
{
    string h1;
    string h2;
    Mat tp1,tp2;
    image1.convertTo(tp1,CV_8UC3);
    image2.convertTo(tp2,CV_8UC3);
    if(type==1)
    {
        h1=aHashValue(tp1);
        h2=aHashValue(tp2);
    }
    else if(type==2)
    {
        h1=pHashValue(tp1);
        h2=pHashValue(tp2);
    }
    else
    {
        h1=dHashValue(tp1);
        h2=dHashValue(tp2);
    }
    int hamming;
    hamming=HanmingDistance(h1,h2);
    return hamming;
}




