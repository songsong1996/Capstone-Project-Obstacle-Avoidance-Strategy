//
// Created by songsong on 19-5-7.
//

#ifndef TEST_CONTOURS_PROCESSOR_H
#define TEST_CONTOURS_PROCESSOR_H

#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include"math.h"
#include "kalman.h"
#include"Hash.h"
#include "SSIM.h"
#include "opticalflow.h"

using namespace std;
using namespace cv;


void computeCentralPoint(vector<Point> &points,Point2d &central_point);

//void ContoursRemoveNoise(double pArea,Mat &image);
//
//void findColorContours(Mat &image1,Mat &depth1,Mat &image2,Mat &depth2);
//
//void matchColorContours();
void preprocessImage(Mat &image);

class obstacle
{
public:
    bool is_dynamic;
    vector<Point> contours_point;
    vector<Point> contours_point_depth;
    vector<Point> position_list;
    Point predict_pt;
    kalman_filter kf;
    Mat color;
    Rect depth_contour;

    Scalar colorS;
    Point2d centralPoint;

    obstacle(){}
    ~obstacle(){}


    obstacle(vector<Point> contours)
    {
        contours_point=contours;
        is_dynamic=true;
        computeCentralPoint(contours_point,centralPoint);
        position_list.push_back(centralPoint);
        kf=kalman_filter(centralPoint);
        colorS=Scalar(rand()%255,rand()%255,rand()%255);
    }

    obstacle(vector<Point> contours,vector<Point> depth_contours,Rect depth_tp,Mat &color_tp)
    {
        contours_point=contours;
        contours_point_depth=depth_contours;
        color_tp.copyTo(color);
        depth_contour=depth_tp;
        is_dynamic=true;
        computeCentralPoint(contours_point,centralPoint);
        position_list.push_back(centralPoint);
        kf=kalman_filter(centralPoint);
        colorS=Scalar(rand()%255,rand()%255,rand()%255);
    }
};


class match
{
public:
    vector<string> file_list;
    vector<obstacle> obstacle_list;
    Mat previous_tpp,previous_color_tpp,previous_depth_tpp;
    Mat present_tpp,present_color_tpp,present_depth_tpp;

    int file_num;
    Mat color_draw,depth_draw;
 //   void match_contours(Mat &previous_top,Mat &previous_color,  Mat &present_top,Mat &present_color) ;
    bool if_stop=false;
  //  void match_contours(Mat &previous_top,  Mat &present_top) ;=

    vector<vector<Point>> contours_filter(Mat &image,vector<vector<Point>> &unfiltered,double arealength);
    vector<Mat> match_contours2(Mat &previous_top,Mat &previous_color, Mat &previous_depth, Mat &present_top,Mat &present_color,Mat &present_depth);

    void delete_same(vector<obstacle> &obstacle_lists);

    void delete_past( vector<obstacle> &obstacle_lists,vector<obstacle> &contours );
    void search_if_exist(vector<vector<Point>> &contours,  vector<obstacle> &obstacle_lists);
    bool if_stop_now(vector<obstacle> &obstacle_lists);
    void search_if_exist(vector<obstacle> &contours,  vector<obstacle> &obstacle_lists);
    double calColorMean(Mat &img0,Mat &img1);

    Mat draw_contours(Mat &present_top, vector<obstacle> &obstacle_lists);
    Mat draw_color(Mat& present_color, vector<obstacle> &obstacle_lists);
    Mat draw_ground(Mat& present_color, Mat& present_depth);

    int find_same2(obstacle &obs,vector<obstacle> &obstacle_lists,int &is_exist);
    Mat convertTo3Channels( Mat &binImg);
    match()
    {
    }
    ~match(){}
   // match(vector<string> file_list_tp);
    match(vector<string> color_list,vector<string> depth_list,vector<string> top_view);


    int find_same(obstacle &obs,vector<obstacle> &obstacle_lists,int &is_exist);
    void draw_single(Mat &image,vector<Point> &points,string windowname, Scalar color,int mode=0);

};

//Point2d linearPredict(obstacle tp);


double calDist(Point2d p1,Point2d p2);
double calDist(Point3d p1, Point3d p2);
void initContourList( vector<obstacle> &obstacle_lists,Mat &color,Mat &depth,Mat &topview,vector<vector<Point>> &contours);
void initContourList( vector<obstacle> &obstacle_lists,Mat &color,Mat &depth,Mat &topview,vector<vector<Point>> &contours_top_view,vector<vector<Point>> &contours_depth);

double cal_contours_top(Mat top,vector<Point> contour);
void sort(Mat depth,vector<vector<Point>> &list,int mode);
double cal_contours_depth(Mat depth,vector<Point> contour);
Mat stressDepth2(Mat &image) ;
Mat getRange(Mat &img,int row1,int row2,int col1,int col2);
Mat getRange2(Mat &img,vector<Point> contour);


vector<double> normalize_list(vector<double> list);

void process_Contours(Mat top_view ,Mat depth, vector<vector<Point>> &contours_top_view, vector<vector<Point>> &contours_depth);
#endif //TEST_CONTOURS_PROCESSOR_H
