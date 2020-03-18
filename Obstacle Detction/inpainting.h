//
// Created by songsong on 19-5-30.
//

#ifndef TEST_INPAINTING_H
#define TEST_INPAINTING_H

#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;


class preprocessor
{
public:
    Mat color;
    Mat depth;
    Mat tmp_color,tmp_depth;
    Mat structure_component;
    int iter_num;
    double dt, epsilon,lambda;

    int radius;
    vector<Point> circle_list(int i0,int j0, int radius);
    double h1,h2,h3;
    int beta;
    double compute_w1(int i0,int j0,int i,int j);
    double compute_w2(Mat depth,int i0,int j0,int i,int j,double d_max);
    double compute_w3(int i0,int j0,int i,int j);
    double compute_dp(Mat depth,int i0,int j0);
    Mat Structure_Fusion();
    Mat Total_Variation(Mat img,int iter);
    preprocessor(Mat col,Mat dep,int iter);
    ~preprocessor(){}
};



Mat Total_Variation2(Mat img,int iter);

#endif //TEST_INPAINTING_H
