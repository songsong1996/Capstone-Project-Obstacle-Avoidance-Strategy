//
// Created by songsong on 19-3-17.
//

#include "outermatch.h"
#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/highgui.hpp>
#include <string>
#include <iostream>
using namespace std;
using namespace cv;

void Outermatch::readCameraMat()
{
//    Mat camK,camDiscoeff;
    camK.convertTo(camK, CV_32FC1);
    camDiscoeff.convertTo(camDiscoeff, CV_32FC1);
    ofstream out;
    out.open(cameraMatPath, ios::out);
    out<<camK.at<float>(0, 0)<<endl;
    out<<camK.at<float>(1, 1)<<endl;
    out<<camK.at<float>(0, 2)<<endl;
    out<<camK.at<float>(1, 2)<<endl;

    out << camDiscoeff.at<float>(0, 0) << endl;
    out << camDiscoeff.at<float>(0, 1) << endl;
    out << camDiscoeff.at<float>(0, 2) << endl;
    out << camDiscoeff.at<float>(0, 3) << endl;
    out << camDiscoeff.at<float>(0, 4) << endl;

    out.close();
}


Mat Outermatch::RectifyImage()
{
    image_left=imread(imageLeftPath,0);
    image_right=imread(imageRightPath,0);
    //判断图像是否加载成功
    if (image_left.empty() || image_right.empty())
    {
        cout << "图像加载失败";
    }
    else
        cout << "图像加载成功..." << endl << endl;
    //检测特征点
    int minHessian = 700;
    vector<KeyPoint>keypoints_object, keypoints_scene;
    Ptr<cv::xfeatures2d::SURF>detector = cv::xfeatures2d::SURF::create(minHessian);

    detector->detect( image_left, keypoints_object);

    detector->detect(image_right, keypoints_scene );

    //计算特征点描述子
    Ptr<cv::xfeatures2d::SURF>extractor = cv::xfeatures2d::SURF::create();

    // Mat descriptors1, descriptors2;
    Mat descriptors_object, descriptors_scene;
    extractor->compute( image_left, keypoints_object, descriptors_object );

    extractor->compute( image_right, keypoints_scene, descriptors_scene );

//    cv::xfeatures2d::SurfDescriptorExtractor extractor;

//    extractor.compute(image_left, keypoints_object, descriptors_object);
//    extractor.compute(image_right, keypoints_scene, descriptors_scene);
    //使用FLANN进行特征点匹配
    FlannBasedMatcher matcher;
    vector<DMatch>matches;
    matcher.match(descriptors_object, descriptors_scene, matches);
    //计算匹配点之间最大和最小距离
    double max_dist = 0;
    double min_dist = 100;
    for (int i = 0; i < descriptors_object.rows; i++)
    {
        double dist = matches[i].distance;
        if (dist < min_dist)
        {
            min_dist = dist;
        }
        else if (dist > max_dist)
        {
            max_dist = dist;
        }
    }
    printf("Max dist: %f \n", max_dist);
    printf("Min dist: %f \n", min_dist);
    //绘制“好”的匹配点
    vector<DMatch>good_matches;
    for (int i = 0; i < descriptors_object.rows; i++)
    {
        if (matches[i].distance<2*min_dist)
        {
            good_matches.push_back(matches[i]);
        }
    }
    Mat image_matches;
    drawMatches(image_left, keypoints_object, image_right, keypoints_scene, good_matches, image_matches, Scalar::all(-1), Scalar::all(-1), vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
    //定位“好”的匹配点
    vector<Point2f> mleft;
    vector<Point2f> mright;
    for (int i = 0; i < good_matches.size(); i++)
    {
        //DMathch类型中queryIdx是指match中第一个数组的索引,keyPoint类型中pt指的是当前点坐标
        mleft.push_back(keypoints_object[good_matches[i].queryIdx].pt);
        mright.push_back(keypoints_scene[good_matches[i].trainIdx].pt);
    }
    Mat H = findHomography(mleft, mright, RANSAC);
    vector<Point2f> obj_corners(4), scene_corners(4);
    obj_corners[0] = Point(0, 0);
    obj_corners[1] = Point(image_left.cols, 0);
    obj_corners[2] = Point(image_left.cols, image_left.rows);
    obj_corners[3] = Point(0, image_left.rows);
    perspectiveTransform(obj_corners, scene_corners, H);
    //绘制角点之间的直线
    line(image_matches, scene_corners[0] + Point2f(image_left.cols, 0), scene_corners[1] + Point2f(image_left.cols, 0), Scalar(0, 0, 255), 2); line(image_matches, scene_corners[1] + Point2f(image_left.cols, 0), scene_corners[2] + Point2f(image_left.cols, 0), Scalar(0, 0, 255), 2);
    line(image_matches, scene_corners[2] + Point2f(image_left.cols, 0), scene_corners[3] + Point2f(image_left.cols, 0), Scalar(0, 0, 255), 2); line(image_matches, scene_corners[3] + Point2f(image_left.cols, 0), scene_corners[0] + Point2f(image_left.cols, 0), Scalar(0, 0, 255), 2);
    //输出图像
    namedWindow("匹配图像", WINDOW_AUTOSIZE);
    imshow("匹配图像", image_matches);
    waitKey(0);

    //计算外部矩阵
    readCameraMat();
    Mat E = cv::findEssentialMat(mleft, mright, camK, RANSAC);
    cv::Mat R1, R2,R,t;
    cv::decomposeEssentialMat(E, R1, R2, t);
    R = R1.clone();
    t = -t.clone();

    Mat Rl, Rr, Pl, Pr, Q;
    cv::stereoRectify(camK, camDiscoeff, camK, camDiscoeff, image_left.size(), R, t, Rl, Rr, Pl, Pr, Q);

    Mat mapLx, mapLy, mapRx, mapRy;
    cv::initUndistortRectifyMap(Pl(cv::Rect(0, 0, 3, 3)), camDiscoeff, Rl, Pl(cv::Rect(0, 0, 3, 3)), image_left.size(), CV_32FC1, mapRx, mapRy);
    cv::remap(image_left, recImgL, mapRx, mapRy, INTER_LINEAR);
    cv::imwrite("data/recConyL.png", recImgL);

    cv::initUndistortRectifyMap(Pr(cv::Rect(0, 0, 3, 3)), camDiscoeff, R2, Pr(cv::Rect(0, 0, 3, 3)), image_right.size(), CV_32FC1, mapRx, mapRy);
    cv::remap(image_right, recImgR, mapRx, mapRy, INTER_LINEAR);
    cv::imwrite("data/recConyR.png", recImgR);

}

Mat Outermatch::SGBM()
{
    int numberOfDisparities = ((image_left.cols / 8) + 15) & -16;
    cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(0, 16, 3);
    sgbm->setPreFilterCap(32);
    int SADWindowSize = 9;
    int sgbmWinSize = SADWindowSize > 0 ? SADWindowSize : 3;
    sgbm->setBlockSize(sgbmWinSize);
    int cn = image_left.channels();
    sgbm->setP1(8 * cn*sgbmWinSize*sgbmWinSize);
    sgbm->setP2(32 * cn*sgbmWinSize*sgbmWinSize);
    sgbm->setMinDisparity(0);
    sgbm->setNumDisparities(numberOfDisparities);
    sgbm->setUniquenessRatio(10);
    sgbm->setSpeckleWindowSize(100);
    sgbm->setSpeckleRange(32);
    sgbm->setDisp12MaxDiff(1);
//    int alg = STEREO_SGBM;
//    if (alg == STEREO_HH)
//        sgbm->setMode(cv::StereoSGBM::MODE_HH);
//    else if (alg == STEREO_SGBM)
//        sgbm->setMode(cv::StereoSGBM::MODE_SGBM);
//    else if (alg == STEREO_3WAY)
//        sgbm->setMode(cv::StereoSGBM::MODE_SGBM_3WAY);
//    sgbm->compute(imgL, imgR, disp);
}
