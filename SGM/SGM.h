//
// Created by songsong on 19-5-31.
//

#ifndef TEST_SGM_H
#define TEST_SGM_H
#pragma once
#include <iostream>
#include <vector>
#include <cmath>
#include <opencv2/   highgui.h>

#include <iostream>
#include <opencv2/opencv.hpp>
#include <algorithm>
using namespace std;
using namespace cv;

typedef float SGM_DP_TYPE;

class SGM
{
public :
    SGM();
    ~SGM();

//�õ�CT����
    int Count1(unsigned int x);
    void CalulateCT(unsigned char *src, int width, int height, unsigned int *dst);
    void hamin(unsigned int *src1, unsigned *src2, int DMAX, int width, int height, float lamada, vector< vector<int> > &cpds);
    //��̬�滮���
    int GetP2(unsigned char G);
    SGM_DP_TYPE min4(SGM_DP_TYPE d1,SGM_DP_TYPE d2,SGM_DP_TYPE d3,SGM_DP_TYPE d4);
    void SGM_DP_L(vector< vector<SGM_DP_TYPE> >& cpds, int width, int height,int disp_N, unsigned int *disp, unsigned char *img_data,vector<vector<SGM_DP_TYPE>>& new_cpds);
    void SGM_DP_R(vector< vector<SGM_DP_TYPE> >& cpds, int width, int height,int disp_N,unsigned int *disp, unsigned char *img_data,vector<vector<SGM_DP_TYPE>>& new_cpds);
    void SGM_DP_LU(vector< vector<SGM_DP_TYPE> >& cpds, int width, int height,int disp_N,unsigned int *disp, unsigned char *img_data,vector<vector<SGM_DP_TYPE>>& new_cpds);
    void SGM_DP_RD(vector< vector<SGM_DP_TYPE> >& cpds, int width, int height,int disp_N,unsigned int *disp, unsigned char *img_data,vector<vector<SGM_DP_TYPE>>& new_cpds);
    void SGM_DP_RU(vector< vector<SGM_DP_TYPE> >& cpds, int width, int height,int disp_N,unsigned int *disp, unsigned char *img_data,vector<vector<SGM_DP_TYPE>>& new_cpds);
    void SGM_DP_LD(vector< vector<SGM_DP_TYPE> >& cpds, int width, int height,int disp_N,unsigned int *disp, unsigned char *img_data,vector<vector<SGM_DP_TYPE>>& new_cpds);
    void SGM_DP_U(vector< vector<SGM_DP_TYPE> >& cpds, int width, int height,int disp_N,unsigned int *disp, unsigned char *img_data,vector<vector<SGM_DP_TYPE>>& new_cpds);
    void SGM_DP_D(vector< vector<SGM_DP_TYPE> >& cpds, int width, int height,int disp_N,unsigned int *disp, unsigned char *img_data,vector<vector<SGM_DP_TYPE>>& new_cpds);
    void SGM_DP_avg2(vector< vector<SGM_DP_TYPE> >& cpds_1,vector<vector<SGM_DP_TYPE>>&cpds_2,vector<vector<SGM_DP_TYPE>>&cpds_12, int width, int height,int disp_N);

    void SGM_DP_min2(vector< vector<SGM_DP_TYPE> >& cpds_1,vector<vector<SGM_DP_TYPE>>&cpds_2,vector<vector<SGM_DP_TYPE>>&cpds_12, int width, int height,int disp_N);

    void SGM_DP_avg4(vector< vector<SGM_DP_TYPE> >& cpds_1,vector<vector<SGM_DP_TYPE>>&cpds_2,vector< vector<SGM_DP_TYPE> >& cpds_3,vector<vector<SGM_DP_TYPE>>&cpds_4,vector<vector<SGM_DP_TYPE>>&cpds_1234, int width, int height,int disp_N);
    void SGM_DP_avg5(vector< vector<SGM_DP_TYPE> >& cpds_1,vector<vector<SGM_DP_TYPE>>&cpds_2,vector< vector<SGM_DP_TYPE> >& cpds_3,vector<vector<SGM_DP_TYPE>>&cpds_4,vector<vector<SGM_DP_TYPE>>&cpds_5,vector<vector<SGM_DP_TYPE>>&cpds_12345, int width, int height,int disp_N);
    void SGM_DP_avg8(vector< vector<SGM_DP_TYPE> >& cpds_1,vector<vector<SGM_DP_TYPE>>&cpds_2,vector< vector<SGM_DP_TYPE> >& cpds_3,vector<vector<SGM_DP_TYPE>>&cpds_4,vector< vector<SGM_DP_TYPE> >& cpds_5,vector<vector<SGM_DP_TYPE>>&cpds_6,vector< vector<SGM_DP_TYPE> >& cpds_7,vector<vector<SGM_DP_TYPE>>&cpds_8,vector<vector<SGM_DP_TYPE>>&cpds_o, int width, int height,int disp_N);
    //�õ��Ӳ�ͼ
    void GetDisp(vector< vector<SGM_DP_TYPE> >& cpds, int width, int height,int disp_N,unsigned int *disp);


//LRcheck
    template <typename T>
    void GetRightDisp(unsigned int* rightdisp, vector<vector<T >> &cost, int width, int height, int DMax)
    {


        for (int i = 0;i<height;i++)
            for (int j = 0;j<width;j++)

            {
                /*int destdisp = 0;
                int mincost = cost[i*width + j][0];
                int secmincost = cost[i*width + j][0];*/
                int maxcurrdisp = ((DMax - 1) - (width - 1 - j))<0 ? (DMax - 1) : (width - 1 - j);
                vector<float> rightcost;
                if (maxcurrdisp == (DMax - 1))
                {
                    for (int d = 0;d<DMax;d++)

                    {

                        //cout << cost[i*width + j + d][d] << endl;


                        T tmp = cost[i*width + j + d][d];
                        rightcost.push_back(tmp);

                    }

                    vector< float >::iterator minist = std::min_element(std::begin(rightcost), std::end(rightcost));
                    unsigned char min_index = minist - std::begin(rightcost);
                    rightdisp[i*width + j] = min_index;
                }
                else
                {
                    int destdisp = 0;
                    int mincost = cost[i*width + j][0];
                    int secmincost = cost[i*width + j][0];

                    for (int k = 0;k<maxcurrdisp - 1;k++)

                    {
                        rightcost.push_back(cost[i*width + j + k][k]);
                    }
                    vector< float >::iterator minist1 = std::min_element(std::begin(rightcost), std::end(rightcost));
                    unsigned char min_index = minist1 - std::begin(rightcost);
                    rightdisp[i*width + j] = min_index;
                }
            }
    }
    template <typename T>
    void LRcheck(int height, int width, T *leftdisp, T *rightdisp, T *LRcheckresult, int DMax)
    {
        for (int row = 0;row<height;row++)
            for (int col = 0;col < width;col++)

            {
                //cout << leftdisp[0] << "   " << rightdisp[0];
                int leftpixelDisp = leftdisp[row*width + col];
                if (row*width + col - leftpixelDisp >= 0)//С��0Ҳ����ƥ���
                {
                    int rightpixelDisp = rightdisp[row*width + col - leftpixelDisp];

                    //	cout << leftdisp[0]<<"   "<< rightdisp[0];
                    int diff = rightpixelDisp - leftpixelDisp;

                    if (abs(diff) > 1)
                    {
                        LRcheckresult[row*width + col] = 0;//��־��ƥ���
                        //	cout << row << "  " << col << endl;

                    }
                    else
                    {
                        LRcheckresult[row*width + col] = leftpixelDisp;
                    }
                }
                else
                    LRcheckresult[row*width + col] = 1;//��־��ƥ���
            }

    }
//��ʾ
    void GenerateFalseMap(cv::Mat &src, cv::Mat &disp);
    void ShowcolorDisp(unsigned int *disp, int width, int height, char * name, char *savename, int DMax);
    void ShowDispgray(unsigned int *disp, int width, int height, char * name, char *savename, int DMax);

//����ͼ�������ͬ������ƥ��ӿ�

    int GetDisprity_mat(cv::Mat& img11,cv::Mat &img22,int path,int DMax,bool debug_display,cv::Mat &LRcheckdisp,cv::Mat &rawdisp);//cv::Mat &rawdisp=nullptr
    int Getdisprity_float(float*left,float*right,int path,int DMax,bool debug_display,int width,int height,float*lrcheckdisp ,float*rawdisp);

//��������
    void uintToMat_8UC1(unsigned int *input,cv::Mat& out ,int height,int width);
    void uintTofloat(unsigned int *input,float* out ,int height,int width);


private  :


    static const	float P1 ;
    static const float P2_apha ;
    static const float P2_gamma;
    static const float P2_min   ;



};


//
//CvStereoGCState* state = cvCreateStereoGCState( 16, 2 );
//left_disp_  =cvCreateMat( left->height,left->width, CV_32F );
//right_disp_ =cvCreateMat( right->height,right->width,CV_32F );
//cvFindStereoCorrespondenceGC( left, right, left_disp_, right_disp_, state, 0 );
//cvReleaseStereoGCState( &state );
//
//
//
//void GC()
//{
//    IplImage * img1 = cvLoadImage("left.png",0);
//    IplImage * img2 = cvLoadImage("right.png",0);
//    CvStereoGCState* GCState=cvCreateStereoGCState(64,3);
//    assert(GCState);
//    cout<<"start matching using GC"<<endl;
//    CvMat* gcdispleft=cvCreateMat(img1->height,img1->width,CV_16S);
//    CvMat* gcdispright=cvCreateMat(img2->height,img2->width,CV_16S);
//    CvMat* gcvdisp=cvCreateMat(img1->height,img1->width,CV_8U);
//    int64 t=getTickCount();
//    cvFindStereoCorrespondenceGC(img1,img2,gcdispleft,gcdispright,GCState);
//    t=getTickCount()-t;
//    cout<<"Time elapsed:"<<t*1000/getTickFrequency()<<endl;
//    //cvNormalize(gcdispleft,gcvdisp,0,255,CV_MINMAX);
//    //cvSaveImage("GC_left_disparity.png",gcvdisp);
//    cvNormalize(gcdispright,gcvdisp,0,255,CV_MINMAX);
//    cvSaveImage("GC_right_disparity.png",gcvdisp);
//
//
//    cvNamedWindow("GC_disparity",0);
//    cvShowImage("GC_disparity",gcvdisp);
//    cvWaitKey(0);
//    cvReleaseMat(&gcdispleft);
//    cvReleaseMat(&gcdispright);
//    cvReleaseMat(&gcvdisp);
//}
//





#endif //TEST_SGM_H
