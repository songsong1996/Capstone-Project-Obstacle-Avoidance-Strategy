//
// Created by songsong on 19-5-17.
//

#ifndef TEST_DHASH_H
#define TEST_DHASH_H

#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;


//均值Hash算法
string aHashValue(Mat &image);

//pHash算法
string pHashValue(Mat &image);

string dHashValue(Mat &image);

//汉明距离计算
int HanmingDistance(string &str1,string &str2);

int HashMatch(Mat &image1,Mat &image2,int type);

#endif //TEST_DHASH_H
