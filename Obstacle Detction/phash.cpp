#include <iostream>
#include <stdio.h>
#include <fstream>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>
using namespace std;
using namespace cv;
int fingerprint(Mat src, Mat* hash);

int main()
{
    Mat src = imread("E:\\image\\image\\image\\person.jpg", 0);
    if(src.empty())
    {
        cout << "the image is not exist" << endl;
        return -1;
    }
    Mat srchash, dsthash;
    fingerprint(src, &srchash);
    for(int i = 1; i <= 8; i++)
    {
        string path0 = "E:\\image\\image\\image\\person";
        string number;
        stringstream ss;
        ss << i;
        ss >> number;
        string path = "E:\\image\\image\\image\\person" + number +".jpg";
        Mat dst = imread(path, 0);
        if(dst.empty())
        {
            cout << "the image is not exist" << endl;
            return -1;
        }
        fingerprint(dst, &dsthash);
        int d = 0;
        for (int n = 0; n < srchash.size[1]; n++)
            if (srchash.at<uchar>(0,n) != dsthash.at<uchar>(0,n)) d++;

        cout <<"person" << i <<"  distance=  " <<d<<"\n";
    }
    system("pause");
    return 0;
}


double phash(Mat src, Mat dst)  //
{
    Mat srchash, dsthash;
    fingerprint(src, &srchash);

        if(dst.empty())
        {
            cout << "the image is not exist" << endl;
            return -1;
        }
        fingerprint(dst, &dsthash);
        int d = 0;
        for (int n = 0; n < srchash.size[1]; n++)
            if (srchash.at<uchar>(0,n) != dsthash.at<uchar>(0,n)) d++;

       // cout <<"person" << i <<"  distance=  " <<d<<"\n";
        return d;
}



int fingerprint(Mat src, Mat* hash)
{
    resize(src, src, Size(32, 32));
    src.convertTo(src, CV_8U);
    Mat srcDCT;
    dct(src, srcDCT);
    srcDCT = abs(srcDCT);
    double sum = 0;
    for (int i = 0; i < 8; i++)
        for (int j = 0; j < 8; j++)
            sum += srcDCT.at<char>(i,j);

    double average = sum/64;
    Mat phashcode= Mat::zeros(Size(8, 8), CV_8U);
    for (int i = 0; i < 8; i++)
        for (int j = 0; j < 8; j++)
            phashcode.at<char>(i,j) = srcDCT.at<char>(i,j) > average ? 1:0;

    *hash = phashcode.reshape(0,1).clone();
    return 0;
}