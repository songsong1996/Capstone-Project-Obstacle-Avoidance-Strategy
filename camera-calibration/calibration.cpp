#include "calibration.h"
#include <iostream>
#include <stdlib.h>
#include <math.h>
using namespace std;

bool CCalibration::writeParams()
{
    camK.convertTo(camK, CV_32FC1);
    camDiscoeff.convertTo(camDiscoeff, CV_32FC1);
    ofstream out;
    out.open(calibResultPath+"calibResult.txt", ios::out);
    out<<camK.at<float>(0, 0)<<endl;
    out<<camK.at<float>(1, 1)<<endl;
    out<<camK.at<float>(0, 2)<<endl;
    out<<camK.at<float>(1, 2)<<endl;
//#ifdef CV
    out << camDiscoeff.at<float>(0, 0) << endl;
    out << camDiscoeff.at<float>(0, 1) << endl;
    out << camDiscoeff.at<float>(0, 2) << endl;
    out << camDiscoeff.at<float>(0, 3) << endl;
    out << camDiscoeff.at<float>(0, 4) << endl;
//#elif defined FISHEYE
//    out << camDiscoeff.at<float>(0, 0) << endl;
//	out << camDiscoeff.at<float>(0, 1) << endl;
//	out << camDiscoeff.at<float>(0, 2) << endl;
//	out << camDiscoeff.at<float>(0, 3) << endl;
//#endif
    out.close();
    return true;
}

bool CCalibration::readPatternImg()
{
    int imgNum=0;
    Mat img;
    do
    {
        stringstream ss;
        ss<<imgNum;
        string path=patternImgPath+ss.str()+".jpg";
        img=imread(path, 0);
        if (!img.data)
        {
            break;
        }
        patternImgList.push_back(img);
        imgNum++;
    } while(true);
    if (imgNum==0)
    {
        cout<<" error! No pattern imgs!"<<endl;
        return false;
    }
    this->imgNum=imgNum;
    imgHeight=patternImgList[0].rows;
    imgWidth=patternImgList[0].cols;

    return true;
}
bool CCalibration::testCorners(vector<cv::Point2f>& corners, int patternWidth, int patternHeight)
{
    if (corners.size() != patternWidth * patternHeight)
    {
        return false;
    }
    double dx1, dx2, dy1, dy2;
    double cosVal;
    for (int i = 0; i < patternHeight; ++i)
    {
        for (int j = 0; j < patternWidth - 2; ++j)
        {
            dx1 = corners[i*patternWidth + j + 1].x - corners[i*patternWidth + j].x;
            dy1 = corners[i*patternWidth + j + 1].y - corners[i*patternWidth + j].y;
            dx2 = corners[i*patternWidth + j + 2].x - corners[i*patternWidth + j + 1].x;
            dy2 = corners[i*patternWidth + j + 2].y - corners[i*patternWidth + j + 1].y;
            cosVal = (dx1 * dx2 + dy1 * dy2) / sqrt((dx1 * dx1 + dy1 * dy1) * (dx2 * dx2 + dy2 * dy2));
            if (fabs(cosVal) < 0.993)
            {
                return false;
            }
        }
    }
    for (int i = 0; i < patternHeight - 2; ++i)
    {
        for (int j = 0; j < patternWidth; ++j)
        {
            dx1 = corners[(i + 1)*patternWidth + j].x - corners[i*patternWidth + j].x;
            dy1 = corners[(i + 1)*patternWidth + j].y - corners[i*patternWidth + j].y;
            dx2 = corners[(i + 2)*patternWidth + j].x - corners[(i + 1)*patternWidth + j].x;
            dy2 = corners[(i + 2)*patternWidth + j].y - corners[(i + 1)*patternWidth + j].y;
            cosVal = (dx1 * dx2 + dy1 * dy2) / sqrt((dx1 * dx1 + dy1 * dy1) * (dx2 * dx2 + dy2 * dy2));
            if (fabs(cosVal) < 0.993)
            {
                return false;
            }
        }
    }
    return true;
}


void CCalibration::init3DPoints(cv::Size boardSize, cv::Size squareSize, vector<cv::Point3f> &singlePatternPoint)
{
    for (int i = 0; i<boardSize.height; i++)
    {
        for (int j = 0; j<boardSize.width; j++)
        {
            cv::Point3f tempPoint;
            tempPoint.x = float(i * squareSize.width);
            tempPoint.y = float(j * squareSize.height);
            tempPoint.z = 0;
            singlePatternPoint.push_back(tempPoint);
        }
    }
}

void CCalibration::calibProcess()
{
    double time0=(double)getTickCount();
    vector<Point2f> corners;
    vector<vector<Point2f>> cornersSeq;
    vector<Mat> image_Seq;
    int successImgNum=0;
    int count=0;
    cout<<"********��ʼ��ȡ�ǵ㣡********"<<endl;
    Mat image,scaleImg;
    for (int i=0; i<imgNum; i++)
    {
        cout<<"Image#"<<i<<"......."<<endl;
//        cv::namedWindow("qq",WINDOW_NORMAL);
        image=patternImgList[i].clone();
//        imshow("qq",image);
//        waitKey(0);
      //  cv::resize(image, scaleImg, cv::Size(), scale, scale, INTER_LINEAR);
//		imshow("wq",scaleImg);
//		waitKey(0);
        bool patternfound= findChessboardCorners(image, boardSize,
                                                 corners, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE+CALIB_CB_FAST_CHECK);
        if (!patternfound)
        {
            cout<<"Can not find chess board corners!\n"<<endl;
            continue;
        }
        else
        {
            for (int num = 0; num < corners.size(); num++)
            {
                cv::Point2f tempPoint = corners[num];
                corners[num] = cv::Point2f(tempPoint.x / scale, tempPoint.y / scale);
            }


            cornerSubPix(image, corners, Size(11, 11), Size(-1,-1) , TermCriteria( 3, 30, 0.1));
            bool good = testCorners(corners, boardSize.width, boardSize.height);
            if (false == good)	continue;
            Mat cornerImg = image.clone();
            cvtColor(cornerImg, cornerImg, COLOR_GRAY2BGR);
            for (int j=0; j< corners.size(); j++)
            {
                circle(cornerImg, corners[j], 20, Scalar(0,0,255), 2, 8, 0);
            }
            namedWindow("CirclePattern",WINDOW_NORMAL);
            imshow("CirclePattern", cornerImg);
            cout << "press any key to see next pattern image" << endl;
            waitKey(1);

            count +=(int)corners.size();
            successImgNum++;
            cornersSeq.push_back(corners);
            image_Seq.push_back(image);
        }
    }
    cout<<"*******�ǵ���ȡ��ɣ�******"<<endl;

    Size squre_size=Size(20,20);
    vector<vector<Point3f>> object_points;
    vector<int> pointCounts;

    init3DPoints(boardSize, squre_size, singlePatternPoints);

    for (int n = 0; n<successImgNum; n++)
    {
        object_points.push_back(singlePatternPoints);
        pointCounts.push_back(boardSize.width * boardSize.height);
    }

    cout<<"*****��ʼ�궨!******"<<endl;
    Size imgSize=Size(imgWidth, imgHeight);
    vector<Vec3d> rotation;
    vector<Vec3d> translation;
    int flags=0;
    cv::calibrateCamera(object_points, cornersSeq, imgSize, camK, camDiscoeff,
                        rotation, translation, flags);
    cout<<"*****�궨��ɣ�*****"<<endl;
    double time1=getTickCount();
    cout<<"Calibration time :"<<(time1-time0)/getTickFrequency()<<"s"<<endl;
    vector<int> outLierIndex;
    evaluateCalibrationResult(object_points, cornersSeq, pointCounts, rotation, translation,
                              camK, camDiscoeff, successImgNum, outLierIndex, errThresh);
    vector<vector<cv::Point2f>> newCornersSeq;
    successImgNum = 0;
    for (int i = 0; i < cornersSeq.size(); i++)
    {
        if (outLierIndex[i] == 0)
        {
            newCornersSeq.push_back(cornersSeq[i]);
            successImgNum++;
        }
    }
    vector<vector<cv::Point3f>> newObjectPoints;
    for (int n = 0; n<successImgNum; n++)
    {
        newObjectPoints.push_back(singlePatternPoints);
    }
    cv::calibrateCamera(object_points, cornersSeq, imgSize, camK, camDiscoeff,
                        rotation, translation, flags);
    outLierIndex.clear();
    evaluateCalibrationResult(object_points, cornersSeq, pointCounts, rotation, translation,
                              camK, camDiscoeff, successImgNum, outLierIndex, errThresh);
//#ifdef DEBUG
//    //ͨ������У��Ч���鿴������궨Ч��
//	cv::Mat R = cv::Mat::eye(3, 3, CV_32FC1);
//	cv::Mat mapx, mapy, newCamK, undistortImg, showImg;
//	cv::initUndistortRectifyMap(camK, camDiscoeff, R, camK, imgSize, CV_32FC1, mapx, mapy);
//	cv::remap(image_Seq[0], undistortImg, mapx, mapy, CV_INTER_LINEAR);
//	cv::resize(undistortImg, showImg, cv::Size(), 0.25, 0.25, CV_INTER_LINEAR);
//	string winName = "undistortImg";
//	cv::namedWindow(winName, 1);
//	cv::imshow(winName, showImg);
//	cv::waitKey(0);
//#endif
}

int CCalibration::evaluateCalibrationResult(vector<vector<cv::Point3f>> objectPoints, vector<vector<cv::Point2f>> cornerSquare, vector<int> pointCnts, vector<cv::Vec3d> _rvec,
                                            vector<cv::Vec3d> _tvec, cv::Mat _K, cv::Mat _D, int count, vector<int> &outLierIndex, int errThresh)
{
    string evaluatPath = calibResultPath + "evaluateCalibrationResult.txt";
    ofstream fout(evaluatPath);

    double total_err = 0.0;
    double err = 0.0;
    vector<cv::Point2f> proImgPoints;
    for (int i = 0; i< count; i++)
    {
        float maxValue = -1;
        vector<cv::Point3f> tempPointSet = objectPoints[i];
        cv::projectPoints(tempPointSet, _rvec[i], _tvec[i], _K, _D, proImgPoints);
        vector<cv::Point2f> tempImgPoint = cornerSquare[i];
        cv::Mat tempImgPointMat = cv::Mat(1, tempImgPoint.size(), CV_32FC2);
        cv::Mat proImgPointsMat = cv::Mat(1, proImgPoints.size(), CV_32FC2);
        for (int j = 0; j != tempImgPoint.size(); j++)
        {
            proImgPointsMat.at<cv::Vec2f>(0, j) = cv::Vec2f(proImgPoints[j].x, proImgPoints[j].y);
            tempImgPointMat.at<cv::Vec2f>(0, j) = cv::Vec2f(tempImgPoint[j].x, tempImgPoint[j].y);
            float dx = proImgPoints[j].x - tempImgPoint[j].x;
            float dy = proImgPoints[j].y - tempImgPoint[j].y;
            float diff = sqrt(dx*dx + dy*dy);
            if (diff > maxValue)
            {
                maxValue = diff;
            }
        }
        fout << " " << i << "  " << maxValue << "  " << endl;

        if (maxValue > errThresh)
        {
            outLierIndex.push_back(-1);
        }
        else
        {
            outLierIndex.push_back(0);
        }
    }
    fout.close();
    return 0;
}
void CCalibration::run()
{
    bool readSuccess=readPatternImg();
    if (!readSuccess)
    {
        cout << "Fail!  No Pattern Imgs !" << endl;
        getchar();
    }
    calibProcess();
    writeParams();
}

