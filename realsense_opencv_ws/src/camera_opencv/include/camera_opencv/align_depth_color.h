//
// Created by songsong on 19-4-16.
//

#ifndef TEST_ALIGN_DEPTH_COLOR_H
#define TEST_ALIGN_DEPTH_COLOR_H

#include<librealsense2/rs.hpp>
#include<librealsense2/rsutil.h>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;

float get_depth_scale(rs2::device dev);
Mat align_Depth2Color(Mat depth,Mat color,rs2::pipeline_profile profile);

Mat SmoothImage(Mat image);
Mat depth_top(Mat depth_align,int rows,int cols,int height_min,int height_max);


Mat stressDepth(Mat &image);







#endif //TEST_ALIGN_DEPTH_COLOR_H
