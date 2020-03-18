//
// Created by songsong on 19-6-3.
//


#include "opencv2/opencv.hpp"
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <string.h>
#include <dirent.h>
#include <unistd.h>
#include <vector>
#include<ctime>
using namespace std;
using namespace cv;
using namespace cv::ml;

////original
//int main()
//{
//    const int MAX_CLUSTERS = 5;
//    Vec3b colorTab[] =
//            {
//                    Vec3b(0, 0, 255),
//                    Vec3b(0, 255, 0),
//                    Vec3b(255, 100, 100),
//                    Vec3b(255, 0, 255),
//                    Vec3b(0, 255, 255)
//            };
//    Mat data, labels;
//    Mat pic = imread("../data/pcd/depth/p0000000243.png");
//    for (int i = 0; i < pic.rows; i++)
//        for (int j = 0; j < pic.cols; j++)
//        {
//            Vec3b point = pic.at<Vec3b>(i, j);
//            Mat tmp = (Mat_<float>(1, 3) << point[0], point[1], point[2]);
//            data.push_back(tmp);
//        }
//
//    int N =6;  //聚成3类
//    Ptr<EM> em_model = EM::create();
//    em_model->setClustersNumber(N);
//    em_model->setCovarianceMatrixType(EM::COV_MAT_SPHERICAL);
//    em_model->setTermCriteria(TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 300, 0.1));
//    em_model->trainEM(data, noArray(), labels, noArray());
//
//    int n = 0;
//    //显示聚类结果，不同的类别用不同的颜色显示
//    for (int i = 0; i < pic.rows; i++)
//        for (int j = 0; j < pic.cols; j++)
//        {
//            int clusterIdx = labels.at<int>(n);
//            pic.at<Vec3b>(i, j) = colorTab[clusterIdx];
//            n++;
//        }
//    imshow("pic", pic);
//    waitKey(0);
//
//    return 0;
//}



//int main()
//{
//    clock_t  starttime,endtime;
//    starttime=clock();
//    Mat pic = imread("../data/pcd/depth/p0000000243.png");
//    Mat top_view=   imread("../data/pcd/top_view/p0000000243.png",IMREAD_UNCHANGED);
//    Mat top_view_tp;
//    Mat element1 = getStructuringElement(MORPH_RECT, Size(17,17));
//    dilate(top_view, top_view_tp, element1);
////    imshow("top",top_view_tp);
////    waitKey(0);
//
//    vector<vector<Point>> contours;
//    vector<Vec4i> hierarchy;
//    findContours(top_view_tp, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
//
//    const int MAX_CLUSTERS = 5;
//    Vec3b colorTab[] =
//            {
//                    Vec3b(0, 0, 255),
//                    Vec3b(0, 255, 0),
//                    Vec3b(255, 100, 100),
//                    Vec3b(255, 0, 255),
//                    Vec3b(0, 255, 255)
//            };
//    Mat data, labels;
//    for (int i = 0; i < pic.rows; i++)
//        for (int j = 0; j < pic.cols; j++)
//        {
//            Vec3b point = pic.at<Vec3b>(i, j);
//            Mat tmp = (Mat_<float>(1, 3) << point[0], point[1], point[2]);
//            data.push_back(tmp);
//        }
//
//    int N =contours.size()+1;  //聚成3类
//    Ptr<EM> em_model = EM::create();
//    em_model->setClustersNumber(N);
//    em_model->setCovarianceMatrixType(EM::COV_MAT_SPHERICAL);
//    em_model->setTermCriteria(TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 300, 0.1));
//    em_model->trainEM(data, noArray(), labels, noArray());
//
//    int n = 0;
//    //显示聚类结果，不同的类别用不同的颜色显示
//    for (int i = 0; i < pic.rows; i++)
//        for (int j = 0; j < pic.cols; j++)
//        {
//            int clusterIdx = labels.at<int>(n);
//            pic.at<Vec3b>(i, j) = colorTab[clusterIdx];
//            n++;
//        }
//    endtime=clock();
//        cout<<"run time="<<(double)(endtime-starttime)/CLOCKS_PER_SEC<<endl;
//    imshow("pic", pic);
//    waitKey(0);
//
//    return 0;
//}




//int main()
//{
//    Mat pic = imread("../data/pcd/depth/p0000000243.png",IMREAD_UNCHANGED);
//    Mat top_view=   imread("../data/pcd/top_view/p0000000243.png",IMREAD_UNCHANGED);
//    Mat top_view_tp;
//    Mat element1 = getStructuringElement(MORPH_RECT, Size(17,17));
//    dilate(top_view, top_view_tp, element1);
////    imshow("top",top_view_tp);
////    waitKey(0);
//
//    vector<vector<Point>> contours;
//    vector<Vec4i> hierarchy;
//    Mat pic_tp;
//    pic.convertTo(pic_tp,CV_8UC1,255.0/65535.0);
//    findContours(pic_tp, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
//
//    for(int k=0;k<contours.size();k++)
//    {
//        double min=255,max=0;
//
//        for(int i=0;i<pic_tp.rows;i++)
//            for(int j=0;j<pic_tp.cols;j++)
//            {
//                if
//            }
//
//    }
//
//
//
//    return 0;
//}



//
//int main( int /*argc*/, char** /*argv*/ )
//{
//    clock_t  starttime,endtime;
//
//    Mat pic = imread("../data/pcd/depth/p0000000243.png");
//    Mat top_view=   imread("../data/pcd/top_view/p0000000243.png",IMREAD_UNCHANGED);
//    Mat pic_tp=Mat::zeros(pic.rows,pic.cols/2,CV_16UC1);
//
//    Mat element1 = getStructuringElement(MORPH_RECT, Size(17,17));
//    dilate(top_view, top_view, element1);
//    for(int i=0;i<pic.rows;i++)
//        for(int j=0;j<pic.cols/2;j++)
//            top_view_tp.at<Vec3b>(i,j)=top_view.at<uchar>()
////    imshow("top",top_view_tp);
////    waitKey(0);
//
//    vector<vector<Point>> contours;
//    vector<Vec4i> hierarchy;
//    findContours(top_view_tp, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
//
//
//
//    Mat data, labels;
//    for (int i = 0; i < pic.rows; i++)
//        for (int j = 0; j < pic.cols; j++)
//        {
//            Vec3b point = pic.at<Vec3b>(i, j);
//            Mat tmp = (Mat_<float>(1, 3) << point[0], point[1], point[2]);
//            data.push_back(tmp);
//        }
//    starttime=clock();
//    int N =contours.size()+1;  //聚成3类
//    Ptr<EM> em_model = EM::create();
//    em_model->setClustersNumber(N);
//    em_model->setCovarianceMatrixType(EM::COV_MAT_SPHERICAL);
//    em_model->setTermCriteria(TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 300, 0.1));
//    em_model->trainEM(data, noArray(), labels, noArray());
//
//    int n = 0;
//    //显示聚类结果，不同的类别用不同的颜色显示
//    for (int i = 0; i < pic.rows; i++)
//        for (int j = 0; j < pic.cols; j++)
//        {
//            int clusterIdx = labels.at<int>(n);
//            pic.at<Vec3b>(i, j) = Vec3b(255.0/(N+2)*(n+1),255.0/(N+2)*(n+1),0);
//            n++;
//        }
//    endtime=clock();
//        cout<<"run time="<<(double)(endtime-starttime)/CLOCKS_PER_SEC<<endl;
//    imshow("pic", pic);
//    waitKey(0);
//
//
//
//
//
//
//    const int MAX_CLUSTERS = 5;
//    Scalar colorTab[] =
//            {
//                    Scalar(0, 0, 255),
//                    Scalar(0,255,0),
//                    Scalar(255,100,100),
//                    Scalar(255,0,255),
//                    Scalar(0,255,255)
//            };
//
//    Mat img(500, 500, CV_8UC3);
//    RNG rng(12345);
//
//    for(;;)
//    {
//        int k, clusterCount = rng.uniform(2, MAX_CLUSTERS+1);
//        int i, sampleCount = rng.uniform(1, 1001);
//        Mat points(sampleCount, 1, CV_32FC2), labels;
//
//        clusterCount = MIN(clusterCount, sampleCount);
//        Mat centers;
//
//        /* generate random sample from multigaussian distribution */
//        for( k = 0; k < clusterCount; k++ )
//        {
//            Point center;
//            center.x = rng.uniform(0, img.cols);
//            center.y = rng.uniform(0, img.rows);
//            Mat pointChunk = points.rowRange(k*sampleCount/clusterCount,
//                                             k == clusterCount - 1 ? sampleCount :
//                                             (k+1)*sampleCount/clusterCount);
//            rng.fill(pointChunk, RNG::NORMAL, Scalar(center.x, center.y), Scalar(img.cols*0.05, img.rows*0.05));
//        }
//
//        randShuffle(points, 1, &rng);
//
//        kmeans(points, clusterCount, labels,
//               TermCriteria( TermCriteria::EPS+TermCriteria::COUNT, 10, 1.0),
//               3, KMEANS_PP_CENTERS, centers);
//
//        img = Scalar::all(0);
//
//        for( i = 0; i < sampleCount; i++ )
//        {
//            int clusterIdx = labels.at<int>(i);
//            Point ipt = points.at<Point2f>(i);
//            circle( img, ipt, 2, colorTab[clusterIdx], FILLED, LINE_AA );
//        }
//
//        imshow("clusters", img);
//
//        char key = (char)waitKey();
//        if( key == 27 || key == 'q' || key == 'Q' ) // 'ESC'
//            break;
//    }
//
//    return 0;
//}



void convert(Mat &pic, Mat contours,int part)
{

    for(int i=0;i<contours.rows-1;i++)
        for(int j=0;j<contours.cols-1;j++)
        {
            int sum1=0,sum2=0;
            for(int k=0;k<3;k++)
            {
                sum1+=abs(contours.at<Vec3b>(i,j)[k]);
                sum1+=abs(contours.at<Vec3b>(i,j)[k]);
                sum2+=abs(contours.at<Vec3b>(i,j+1)[k]);
                sum2+=abs(contours.at<Vec3b>(i+1,j)[k]);
            }

            if(abs(sum2-sum1)>=20)
            {
                if(part==0) //left part
                {
                    pic.at<ushort>(i,j)=0;
                    pic.at<ushort>(i+1,j+1)=0;
                    pic.at<ushort>(i,j+1)=0;
                    pic.at<ushort>(i+1,j)=0;
                } else
                {
                    pic.at<ushort>(i,j+pic.cols/2)=0;
                    pic.at<ushort>(i+1,j+1+pic.cols/2)=0;
                    pic.at<ushort>(i,j+1+pic.cols/2)=0;
                    pic.at<ushort>(i+1,j+pic.cols/2)=0;
                }
            }
        }
}





void process_depth(Mat &pic,Mat &top_view,int part)
{
    clock_t  starttime,endtime;

//    Mat element1 = getStructuringElement(MORPH_RECT, Size(17,17));
//    Mat top_view1;
//    dilate(top_view, top_view1, element1);
//    imshow("top",top_view1);
//    waitKey(0);

    Mat element1 = getStructuringElement(MORPH_RECT, Size(3,3));
    Mat top_view1;
    dilate(top_view, top_view1, element1);

    vector<vector<Point>> contours1,contours2;
    vector<Vec4i> hierarchy1,hierarchy2;
    findContours(top_view1, contours1, hierarchy1, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    for(int k=0;k<contours1.size();k++)
    {
        if(contourArea((contours1[k]))<60)
        {
            for(int i=0;i<top_view1.rows;i++)
                for(int j=0;j<top_view1.cols;j++)
                    if(  pointPolygonTest(contours1[k], Point2f(j, i), false) !=-1)
                        top_view1.at<uchar>(i,j)=0;
        }

    }
    element1 = getStructuringElement(MORPH_RECT, Size(8,8));
    dilate(top_view1, top_view1, element1);
    findContours(top_view1, contours2, hierarchy2, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    cout<<contours2.size()<<endl;
//    imshow("top",top_view1);
//    waitKey(0);


    Mat pic_tp=Mat::zeros(pic.rows,pic.cols/2,CV_8UC3);
    Mat top_view_tp=Mat::zeros(top_view1.rows,top_view1.cols/2,CV_8UC1);
    Mat picc;
    vector<Mat> channels;
    Mat pic_8;
    pic.convertTo(pic_8,CV_8UC1,255.0/65535.0);
    for(int i=0;i<3;i++)
        channels.push_back(pic_8);
    merge(channels,picc);


    if(part==0)   //left part
    {

        for(int i=0;i<pic.rows;i++)
            for(int j=0; j<pic.cols/2;j++)
            {
                pic_tp.at<Vec3b>(i,j)=picc.at<Vec3b>(i,j);

            }

        for(int i=0;i<top_view1.rows;i++)
            for(int j=0;j<top_view1.cols/2;j++)
            {
                top_view_tp.at<uchar>(i,j)=top_view1.at<uchar>(i,j);
            }

    } else  //right part
    {
        for(int i=0;i<pic.rows;i++)
            for(int j=pic.cols/2;j<pic.cols;j++)
            {
                pic_tp.at<Vec3b>(i,j-pic.cols/2)=picc.at<Vec3b>(i,j);

            }

        for(int i=0;i<top_view1.rows;i++)
            for(int j=top_view1.cols/2;j<top_view1.cols;j++)
            {
                top_view_tp.at<uchar>(i,j-top_view1.cols/2)=top_view1.at<uchar>(i,j);
            }

    }

//    imshow("top",pic_tp);
//    waitKey(0);

    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(top_view_tp, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    const int MAX_CLUSTERS = 8;
    Vec3b colorTab[] =
            {
                    Vec3b(0, 0, 255),
                    Vec3b(0, 255, 20),
                    Vec3b(155,40,100),
                    Vec3b(60, 0, 255),
                    Vec3b(40,140,155),
                    Vec3b(100,205,50),
                    Vec3b(255, 0, 120),
                    Vec3b(140, 0, 255)

            };
    starttime=clock();
    Mat data, labels;
    for (int i = 0; i < pic_tp.rows; i++)
        for (int j = 0; j < pic_tp.cols; j++)
        {
            Vec3b point = pic_tp.at<Vec3b>(i, j);
            Mat tmp = (Mat_<float>(1, 3) << point[0], point[1], point[2]);
            data.push_back(tmp);
        }

    int N =contours.size()+1;  //聚成3类
    Ptr<EM> em_model = EM::create();
    em_model->setClustersNumber(N);
    em_model->setCovarianceMatrixType(EM::COV_MAT_SPHERICAL);
    em_model->setTermCriteria(TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 50, 2.0));
    em_model->trainEM(data, noArray(), labels, noArray());
    endtime=clock();
    int n = 0;
    //显示聚类结果，不同的类别用不同的颜色显示
    for (int i = 0; i < pic_tp.rows; i++)
        for (int j = 0; j < pic_tp.cols; j++)
        {
            int clusterIdx = labels.at<int>(n);
            pic_tp.at<Vec3b>(i, j) = colorTab[clusterIdx];
            n++;
        }

    convert(pic,pic_tp,part);

    cout<<"run time="<<(double)(endtime-starttime)/CLOCKS_PER_SEC<<endl;
//    imshow("pic", pic_tp);
//    waitKey(0);
}

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


//int main()
//{
//    clock_t  starttime,endtime;
//
//    string basePath1="../data/pcd/depth/";
//    string basePath2="../data/pcd/top_view/";
//    vector<string> file_list1,file_list2;
//    readFileList(basePath1,file_list1);
//    readFileList(basePath2,file_list2);
//
//for(int ff=0;ff<file_list1.size();ff++)
//  //  for(int ff=18;ff<20;ff++)
//    {
//        Mat pic = imread(file_list1[ff],IMREAD_UNCHANGED);
//        Mat top_view = imread(file_list2[ff],IMREAD_UNCHANGED);
//        for(int i=0;i<2;i++)
//        {
//            process_depth(pic,top_view,i);
//        }
////        imshow("pp",pic);
////        waitKey(10);
//
//        string str=file_list1[ff].substr(file_list1[ff].find_last_of("/")+1,11)   ;
//        imwrite("../data/pcd/depth2/"+ str+".png",pic);
//    }
//    return 0;
//}


int main()
{
    string basePath1="../data/pcd/depth2/";
    string basePath2="../data/pcd/color/";
    vector<string> file_list1,file_list2;
    readFileList(basePath1,file_list1);
    readFileList(basePath2,file_list2);


      for(int k=0;k<file_list1.size();k++)
    //for(int k=18;k<20;k++)
    {
       // k=18;
        Mat depth=imread(file_list1[k],IMREAD_UNCHANGED);
        depth.convertTo(depth,CV_16UC1);
        vector<Mat> channels;
        split(depth,channels);

        Mat color=imread(file_list2[k],IMREAD_UNCHANGED);
        Mat color2=Mat::zeros(depth.size(),CV_8UC3);
        Mat tpp=Mat::zeros(depth.size(),CV_16UC1);
        for(int i=0;i<depth.rows;i++)
            for(int j=0;j<depth.cols;j++)
            {
                tpp.at<ushort>(i,j)=depth.at<ushort>(i,j);
                if(depth.at<ushort>(i,j)!=0 )
                {
                    for(int kk=0;kk<3;kk++)
                        color2.at<Vec3b>(i,j)[kk]=color.at<Vec3b>(i,j)[kk];
                }
            }
//        imshow("d",tpp);
//        imshow("c",color2);
//        waitKey(0);
        string str=file_list1[k].substr(file_list1[k].find_last_of("/")+1,11)   ;
        imwrite("../data/pcd/color2/"+ str+".png",color2);
    }
}

//int main( int /*argc*/, char** /*argv*/ )
//{
//    clock_t  starttime,endtime;
//
//    Mat pic = imread("../data/pcd/depth/p0000000243.png",IMREAD_UNCHANGED);
//    Mat top_view =   imread("../data/pcd/top_view/p0000000243.png",IMREAD_UNCHANGED);
//
//    Mat element1 = getStructuringElement(MORPH_RECT, Size(17,17));
//    dilate(top_view, top_view, element1);
//
//    Mat pic_tp=Mat::zeros(pic.rows,pic.cols/2,CV_8UC3);
//    Mat top_view_tp=Mat::zeros(top_view.rows,top_view.cols/2,CV_8UC1);
//
//    for(int i=0;i<pic.rows;i++)
//        for(int j=0;j<pic.cols/2;j++)
//        {
//            pic_tp.at<Vec3b>(i,j)=Vec3b(pic.at<ushort>(i,j),pic.at<ushort>(i,j),pic.at<ushort>(i,j));
//            top_view_tp.at<uchar>(i,j)=top_view.at<uchar>(i,j);
//        }
//
////    imshow("top",top_view_tp);
////    waitKey(0);
//
//    vector<vector<Point>> contours;
//    vector<Vec4i> hierarchy;
//    findContours(top_view, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
//
//
//
//        const int MAX_CLUSTERS = 5;
//    Vec3b colorTab[] =
//            {
//                    Vec3b(0, 0, 255),
//                    Vec3b(0, 255, 0),
//                    Vec3b(255, 100, 100),
//                    Vec3b(255, 0, 255),
//                    Vec3b(0, 255, 255)
//            };
//    starttime=clock();
//    Mat data, labels;
//    for (int i = 0; i < pic_tp.rows; i++)
//        for (int j = 0; j < pic_tp.cols; j++)
//        {
//            Vec3b point = pic_tp.at<Vec3b>(i, j);
//            Mat tmp = (Mat_<float>(1, 3) << point[0], point[1], point[2]);
//            data.push_back(tmp);
//        }
//
//    int N =contours.size()+1;  //聚成3类
//    Ptr<EM> em_model = EM::create();
//    em_model->setClustersNumber(N);
//    em_model->setCovarianceMatrixType(EM::COV_MAT_SPHERICAL);
//    em_model->setTermCriteria(TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 300, 0.1));
//    em_model->trainEM(data, noArray(), labels, noArray());
//
//    int n = 0;
//    //显示聚类结果，不同的类别用不同的颜色显示
//    for (int i = 0; i < pic_tp.rows; i++)
//        for (int j = 0; j < pic_tp.cols; j++)
//        {
//            int clusterIdx = labels.at<int>(n);
//            pic_tp.at<Vec3b>(i, j) = colorTab[clusterIdx];
//            n++;
//        }
//    endtime=clock();
//        cout<<"run time="<<(double)(endtime-starttime)/CLOCKS_PER_SEC<<endl;
//    imshow("pic", pic_tp);
//    waitKey(0);
//
//    return 0;
//}


//int main( int /*argc*/, char** /*argv*/ )
//{
//    Mat pic = imread("../data/pcd/depth/p0000000243.png",IMREAD_UNCHANGED);
//    Mat pic_tp;
//    pic.convertTo(pic_tp,CV_8UC1,255.0/65535.0);
//    vector<vector<Point>> contours;
//    vector<Vec4i> hierarchy;
//    findContours(pic_tp, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
//
//
//    ushort max=0,min=65535;
//
//    for(int i=0;i<pic.rows;i++)
//        for(int j=0;j<pic.cols-1;j++)
//        {
//            if( abs()   )
//        }
//
//    for(int k=0;k<contours.size();k++)
//    {
//        Rect rec=boundingRect(contours[k]);
//
//        for(int i=rec.y;i<rec.y+rec.height;i++)
//            for(int j=rec.x;j<rec.x+rec.width-1;j++)
//            {
//                if (abs(pic.at<ushort>(i, j)-pic.at<ushort>(i, j+1)) < min)min = abs(pic.at<ushort>(i, j)-pic.at<ushort>(i, j+1));
//                if (abs(pic.at<ushort>(i, j)-pic.at<ushort>(i, j+1)) > max)max = abs(pic.at<ushort>(i, j)-pic.at<ushort>(i, j+1));
//            }
//
//        for(int i=rec.y;i<rec.y+rec.height-1;i++)
//            for(int j=rec.x;j<rec.x+rec.width;j++)
//            {
//                if (abs(pic.at<ushort>(i+1, j)-pic.at<ushort>(i, j)) < min)min = abs(pic.at<ushort>(i+1, j)-pic.at<ushort>(i, j));
//                if (abs(pic.at<ushort>(i+1, j)-pic.at<ushort>(i, j)) > max)max = abs(pic.at<ushort>(i+1, j)-pic.at<ushort>(i, j));
//            }
//        vector<int> border_t=vector<int>((int)(max-min+1));
//
//        for(int ii=0;ii<(int)(max-min);ii++)
//        {
//            border_t[ii]=0;
//        }
//
//
//        for(int i=rec.y;i<rec.y+rec.height;i++)
//            for(int j=rec.x;j<rec.x+rec.width-1;j++)
//            {
//                border_t[ int(abs(pic.at<ushort>(i,j)-min)) ] +=1;
//            }
//
//        for(int i=rec.y;i<rec.y+rec.height-1;i++)
//            for(int j=rec.x;j<rec.x+rec.width;j++)
//            {
//                border_t[int(abs(pic.at<ushort>(i,j)-min)) ]   +=1;
//            }
//
//
//        string str="../data/pcd/p243"+to_string(k)+".txt";
//        ofstream outFile(str.c_str(), ios::app);
//        for(int ii=0;ii<border_t.size();ii++)
//            if(border_t[ii]!=0)
//                outFile <<ii  <<"\t"<<border_t[ii] << endl;//换行
//        outFile.close();
//    }
//
//
//
//    // Mat top_view=   imread("../data/pcd/top_view/p0000000243.png",IMREAD_UNCHANGED);
////    Mat top_view_tp;
////    Mat element1 = getStructuringElement(MORPH_RECT, Size(17,17));
////    dilate(top_view, top_view_tp, element1);
////    imshow("top",top_view_tp);
////    waitKey(0);
//}