//
// Created by songsong on 19-5-31.
//


#include "opencv2/opencv.hpp"
using namespace std;
using namespace cv;
int main(int argc, char* argv[])
{
    Mat left = imread("../data/left.png", IMREAD_GRAYSCALE);
    Mat right = imread("../data/right.png", IMREAD_GRAYSCALE);
    Mat disp;
    int mindisparity = 0;
    int ndisparities = 64;
    int SADWindowSize = 11;
    //SGBM
    cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(mindisparity, ndisparities, SADWindowSize);
    int P1 = 16 * left.channels() * SADWindowSize* SADWindowSize;
    int P2 = 32* left.channels() * SADWindowSize* SADWindowSize;
    sgbm->setP1(P1);
    sgbm->setP2(P2);
    sgbm->setPreFilterCap(5);
    sgbm->setUniquenessRatio(10);
    sgbm->setSpeckleRange(2);

    sgbm->setSpeckleWindowSize(200);
    sgbm->setDisp12MaxDiff(1);
    //sgbm->setMode(cv::StereoSGBM::MODE_HH);
    sgbm->compute(left, right, disp);
    disp.convertTo(disp, CV_32F, 1.0 / 16);                //除以16得到真实视差值
    Mat disp8U = Mat(disp.rows, disp.cols, CV_8UC1);       //显示
    normalize(disp, disp8U, 0, 255, NORM_MINMAX, CV_8UC1);
    imshow("tp",disp8U);
    waitKey(0);
    imwrite("../data/SGBM.png", disp8U);
    return 0;
}


