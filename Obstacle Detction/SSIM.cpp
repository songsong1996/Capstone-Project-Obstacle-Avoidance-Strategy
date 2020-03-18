//
// Created by songsong on 19-6-7.
//


#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<iostream>
#include "SSIM.h"

using namespace std;
using namespace cv;
Scalar getMSSIM( const Mat& i1, const Mat& i2)
{
    const double C1 = 6.5025, C2 = 58.5225;
    /***************************** INITS **********************************/
    int d     = CV_32F;


    Mat I1, I2;
    i1.convertTo(I1, d);           // cannot calculate on one byte large values
    i2.convertTo(I2, d);


    Mat I2_2   = I2.mul(I2);        // I2^2
    Mat I1_2   = I1.mul(I1);        // I1^2
    Mat I1_I2  = I1.mul(I2);        // I1 * I2


    /***********************PRELIMINARY COMPUTING ******************************/


    Mat mu1, mu2;   //
    GaussianBlur(I1, mu1, Size(11, 11), 1.5);
    GaussianBlur(I2, mu2, Size(11, 11), 1.5);


    Mat mu1_2   =   mu1.mul(mu1);
    Mat mu2_2   =   mu2.mul(mu2);
    Mat mu1_mu2 =   mu1.mul(mu2);


    Mat sigma1_2, sigma2_2, sigma12;


    GaussianBlur(I1_2, sigma1_2, Size(11, 11), 1.5);
    sigma1_2 -= mu1_2;


    GaussianBlur(I2_2, sigma2_2, Size(11, 11), 1.5);
    sigma2_2 -= mu2_2;


    GaussianBlur(I1_I2, sigma12, Size(11, 11), 1.5);
    sigma12 -= mu1_mu2;


    ///////////////////////////////// FORMULA ////////////////////////////////
    Mat t1, t2, t3;


    t1 = 2 * mu1_mu2 + C1;
    t2 = 2 * sigma12 + C2;
    t3 = t1.mul(t2);              // t3 = ((2*mu1_mu2 + C1).*(2*sigma12 + C2))


    t1 = mu1_2 + mu2_2 + C1;
    t2 = sigma1_2 + sigma2_2 + C2;
    t1 = t1.mul(t2);               // t1 =((mu1_2 + mu2_2 + C1).*(sigma1_2 + sigma2_2 + C2))


    Mat ssim_map;
    divide(t3, t1, ssim_map);      // ssim_map =  t3./t1;


    Scalar mssim = mean( ssim_map ); // mssim = average of ssim map
    return mssim;
}

double SSIM(Mat im1,Mat im2)
{
    int rows=im1.rows>=im2.rows? im1.rows:im2.rows;
    int cols=im1.cols>=im2.cols?im1.cols:im2.cols;
//    im1.resize(rows,cols);
//    im2.resize(rows,cols);
    resize(im1,im1,Size(cols,rows));
    resize(im2,im2,Size(cols,rows));


    Scalar SSM1=getMSSIM(im1,im2);
    double resu=SSM1.val[0]+SSM1.val[1]+SSM1.val[2];
    resu=resu/3.0;
    return resu;
}

//int main()
//{
//    Mat im1=imread("../data/tp/120.png",IMREAD_UNCHANGED);
//    Mat im2=imread("../data/tp/121.png",IMREAD_UNCHANGED);
//    Mat im3=imread("../data/tp/123.png",IMREAD_UNCHANGED);
//
//    Scalar S1=SSIM(im1,im2);
//    Scalar S2=SSIM(im1,im3);
//return 0;
//
//}






