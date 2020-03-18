//
// Created by songsong on 19-5-30.
//


#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include "inpainting.h"

using namespace std;
using namespace cv;

int main( int argc, char** argv )
{
    Mat img=imread("../data/camera/color/475.png",IMREAD_UNCHANGED);
    vector<Mat> channels,channels_struct;
    vector<Mat> channels_texture;
    Mat tp;
    img.copyTo(tp);
    split(tp,channels);
    for(int i=0;i<3;i++)
    {
        Mat layer= Total_Variation2(channels[i],5);
        channels_struct.push_back(layer);
        Mat text=Mat::zeros(img.rows,img.cols,CV_8UC1);
        for(int ii=0;ii<img.rows;ii++)
            for(int jj=0;jj<img.cols;jj++)
                text.at<uchar>(ii,jj)=255-abs(layer.at<uchar>(ii,jj)-channels[i].at<uchar>(ii,jj))*10;
       // text=layer-channels[i];
        channels_texture.push_back(text);
    }

    Mat img_result=Mat::zeros(img.rows,img.cols,CV_8UC3);
    Mat img_texture=Mat::zeros(img.rows,img.cols,CV_8UC3);
    merge(channels_struct,img_result);
    merge(channels_texture,img_texture);

    imshow("tpppp",img);
    imshow("tp",img_result);
    imshow("tpp",img_texture);
    waitKey(0);

}


Mat stressDepth(Mat &image)
{
    Mat result = Mat::zeros(image.rows, image.cols, CV_16UC1);
    for (int i = 0; i < image.rows; i++)
        for (int j = 0; j < image.cols; j++) {
            if(image.at<uint16_t>(i,j)>450)result.at<uint16_t >(i,j)=0;
            result.at<uint16_t>(i, j) = image.at<uint16_t>(i, j) *100.0;
        }
    return result;
}


//
//int main( int argc, char** argv )
//{
//    Mat color=imread("../data/camera/color/460.png",IMREAD_UNCHANGED);
//   Mat depth=imread("../data/camera/460.png",IMREAD_UNCHANGED);
//
//   depth.convertTo(depth,CV_16UC1);
//   color.convertTo(color,CV_8UC3);
//   preprocessor tp(color,depth,1);
//
//   Mat result=tp.Structure_Fusion();
//   result=stressDepth(result);
//
//    waitKey(0);
//}