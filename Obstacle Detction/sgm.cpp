

#include "opencv2/opencv.hpp"
#include "align_depth_color.h"
using namespace std;
using namespace cv;



void disp2Depth(cv::Mat dispMap, cv::Mat &depthMap)
{
    int type = dispMap.type();

//    float fx = K.at<float>(0, 0);
//    float fy = K.at<float>(1, 1);
//    float cx = K.at<float>(0, 2);
//    float cy = K.at<float>(1, 2);


    float fx = 718.856;
    float  fy = 718.856;
    float  cx = 607.1928;
    float   cy = 185.2157;
    float baseline = 540; //基线距离65mm

    if (type == CV_8UC1)
    {
        const float PI = 3.14159265358;
        int height = dispMap.rows;
        int width = dispMap.cols;

        uchar* dispData = (uchar*)dispMap.data;
        ushort* depthData = (ushort*)depthMap.data;
        for (int i = 0; i < height; i++)
        {
            for (int j = 0; j < width; j++)
            {
                int id = i*width + j;
                if (!dispData[id])  continue;  //防止0除
                depthData[id] = ushort( (float)fx *baseline / ((float)dispData[id]) );
            }
        }
      cv::imshow("tp",depthMap);
        cv::waitKey(0);
    }
    else
    {
        cout << "please confirm dispImg's type!" << endl;
        cv::waitKey(0);
    }
}

int main(int argc, char* argv[])
{
    Mat left = imread("../data/left1.png", IMREAD_UNCHANGED);
    Mat right = imread("../data/right1.png", IMREAD_UNCHANGED);
    Mat disp;
    int mindisparity = 0;
    int ndisparities = 64;
    int SADWindowSize = 11;
    //SGBM
    cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(mindisparity, ndisparities, SADWindowSize);
    int P1 = 32 * left.channels() * SADWindowSize* SADWindowSize;
    int P2 = 64* left.channels() * SADWindowSize* SADWindowSize;
    sgbm->setP1(P1);
    sgbm->setP2(P2);
    sgbm->setPreFilterCap(3);
    sgbm->setUniquenessRatio(10);
    sgbm->setSpeckleRange(2);

    sgbm->setSpeckleWindowSize(100);
    sgbm->setDisp12MaxDiff(1);
    //sgbm->setMode(cv::StereoSGBM::MODE_HH);
    sgbm->compute(left, right, disp);
    disp.convertTo(disp, CV_32F, 1.0 / 16);                //除以16得到真实视差值
    Mat disp8U = Mat(disp.rows, disp.cols, CV_8UC1);       //显示
    normalize(disp, disp8U, 0, 255, NORM_MINMAX, CV_8UC1);
    imshow("tp",disp8U);
    waitKey(0);


    Mat depth=Mat(disp8U.rows,disp8U.cols,CV_16UC1);

    disp2Depth( disp8U,depth);

    depth=SmoothImage(depth);
//    cout<<depth.at<ushort>(depth.rows-50,depth.cols-100)<<endl;
//
//   Mat depth_tp= stressDepth2(depth);
//    imshow("tpp",depth_tp);
////   Mat tp_view;
////   tp_view= depth_top2(depth,1100,depth.cols,(int)(depth.rows/2),(int)(depth.rows*3/4));
//   //imshow("top",tp_view);
//   waitKey(0);

    imwrite("../data/SGBM.png", disp8U);
    imwrite("../data/depthtp.png",depth);
    return 0;
}


