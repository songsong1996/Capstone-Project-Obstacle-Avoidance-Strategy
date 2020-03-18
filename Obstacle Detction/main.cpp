//
// Created by songsong on 19-4-16.
//

#include<librealsense2/rs.hpp>
#include<librealsense2/rsutil.h>
#include"align_depth_color.h"
// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <unistd.h>
#include "contours_processor.h"
#include<iostream>
using namespace cv;
using namespace std;



int main(int argc, char **argv)
{
    const char* depth_win="depth_Image";
    namedWindow(depth_win,WINDOW_AUTOSIZE);
    const char* color_win="color_Image";
    namedWindow(color_win,WINDOW_AUTOSIZE);
    const char* top_win="top_View";
    namedWindow(top_win,WINDOW_AUTOSIZE);

    //深度图像颜色map
    rs2::colorizer c;                          // Helper to colorize depth images

    //创建数据管道
    rs2::pipeline pipe;
    rs2::config pipe_config;
    pipe_config.enable_stream(RS2_STREAM_DEPTH,640,480,RS2_FORMAT_Z16,30);
    pipe_config.enable_stream(RS2_STREAM_COLOR,640,480,RS2_FORMAT_BGR8,30);

    //start()函数返回数据管道的profile
    rs2::pipeline_profile profile = pipe.start(pipe_config);

    //定义一个变量去转换深度到距离
    float depth_clipping_distance = 1.f;

    //声明数据流
    auto depth_stream=profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
    auto color_stream=profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();

    //获取内参
    auto intrinDepth=depth_stream.get_intrinsics();
    auto intrinColor=color_stream.get_intrinsics();

    //直接获取从深度摄像头坐标系到彩色摄像头坐标系的欧式变换矩阵
    auto  extrinDepth2Color=depth_stream.get_extrinsics_to(color_stream);


//    int ii=0;
//    while(ii==0)
    int ii=0;


    Mat previous_depth,previous_color,present_depth,present_color,previous,present;
    match new_folder=match();
    if(getWindowProperty(depth_win,WND_PROP_VISIBLE )&&getWindowProperty(color_win,WND_PROP_VISIBLE ))
    {
        //ii++;
        //堵塞程序直到新的一帧捕获
        rs2::frameset frameset = pipe.wait_for_frames();
        //取深度图和彩色图
        rs2::frame color_frame = frameset.get_color_frame();//processed.first(align_to);
        rs2::frame depth_frame = frameset.get_depth_frame();
        rs2::frame depth_frame_4_show = frameset.get_depth_frame().apply_filter(c);
        //获取宽高
        const int depth_w=depth_frame.as<rs2::video_frame>().get_width();
        const int depth_h=depth_frame.as<rs2::video_frame>().get_height();
        const int color_w=color_frame.as<rs2::video_frame>().get_width();
        const int color_h=color_frame.as<rs2::video_frame>().get_height();

        Mat depth_image(Size(depth_w,depth_h),
                        CV_16U,(void*)depth_frame.get_data(),Mat::AUTO_STEP);
        Mat depth_image_4_show(Size(depth_w,depth_h),
                               CV_8UC3,(void*)depth_frame_4_show.get_data(),Mat::AUTO_STEP);
        Mat color_image(Size(color_w,color_h),
                        CV_8UC3,(void*)color_frame.get_data(),Mat::AUTO_STEP);

//        Mat depth_image=imread("/home/songsong/Documents/private/Graduation-design/codes/Intelrealsense/1/data/t1_Depth.png",IMREAD_UNCHANGED );
//        Mat color_image=imread("/home/songsong/Documents/private/Graduation-design/codes/Intelrealsense/1/data/t1_Color.png",IMREAD_COLOR  );
//        depth_image.convertTo(depth_image,CV_16U );
//        color_image.convertTo(color_image,CV_8UC3);
//
//        cvtColor(depth_image,depth_image,CV_16U);
//        cvtColor(color_image,color_image,CV_8UC3);
        //实现深度图对齐到彩色图
        Mat result=align_Depth2Color(depth_image,color_image,profile);
        result=SmoothImage(result);
        Mat top_view= depth_top(result,600,result.cols,0,result.rows);
//      PointCloud::Ptr pointcloud= image2PointCloud( color_image, depth_image, camera_intrinsic );

        Mat depth1_tp,depth2_tp;
        depth1_tp=stressDepth(depth_image);
        depth2_tp=stressDepth(result);

        //显示
        imshow(depth_win,depth1_tp);
        imshow(color_win,color_image);
        imshow("result",depth2_tp);
        imshow(top_win,top_view);
        waitKey(100);
//        sleep(1000);



    }
    while (getWindowProperty(depth_win,WND_PROP_VISIBLE )&&getWindowProperty(color_win,WND_PROP_VISIBLE )) // Application still alive?
    {
        //ii++;
        //堵塞程序直到新的一帧捕获
        rs2::frameset frameset = pipe.wait_for_frames();
        //取深度图和彩色图
        rs2::frame color_frame = frameset.get_color_frame();//processed.first(align_to);
        rs2::frame depth_frame = frameset.get_depth_frame();
        rs2::frame depth_frame_4_show = frameset.get_depth_frame().apply_filter(c);
        //获取宽高
        const int depth_w=depth_frame.as<rs2::video_frame>().get_width();
        const int depth_h=depth_frame.as<rs2::video_frame>().get_height();
        const int color_w=color_frame.as<rs2::video_frame>().get_width();
        const int color_h=color_frame.as<rs2::video_frame>().get_height();

        Mat depth_image(Size(depth_w,depth_h),
                        CV_16U,(void*)depth_frame.get_data(),Mat::AUTO_STEP);
        Mat depth_image_4_show(Size(depth_w,depth_h),
                               CV_8UC3,(void*)depth_frame_4_show.get_data(),Mat::AUTO_STEP);
        Mat color_image(Size(color_w,color_h),
                        CV_8UC3,(void*)color_frame.get_data(),Mat::AUTO_STEP);

//        Mat depth_image=imread("/home/songsong/Documents/private/Graduation-design/codes/Intelrealsense/1/data/t1_Depth.png",IMREAD_UNCHANGED );
//        Mat color_image=imread("/home/songsong/Documents/private/Graduation-design/codes/Intelrealsense/1/data/t1_Color.png",IMREAD_COLOR  );
//        depth_image.convertTo(depth_image,CV_16U );
//        color_image.convertTo(color_image,CV_8UC3);
//
//        cvtColor(depth_image,depth_image,CV_16U);
//        cvtColor(color_image,color_image,CV_8UC3);
        //实现深度图对齐到彩色图
        Mat result=align_Depth2Color(depth_image,color_image,profile);
        result=SmoothImage(result);
        Mat top_view= depth_top(result,600,result.cols,0,result.rows);
//      PointCloud::Ptr pointcloud= image2PointCloud( color_image, depth_image, camera_intrinsic );

        Mat depth1_tp,depth2_tp;
        depth1_tp=stressDepth(depth_image);
        depth2_tp=stressDepth(result);

        //显示
//        imshow(depth_win,depth1_tp);
//        imshow(color_win,color_image);
//        imshow("result",depth2_tp);
//        imshow(top_win,top_view);
//        waitKey(100);
        //   sleep(1000);

        top_view.copyTo(present);
        color_image.copyTo(present_color);
        depth2_tp.copyTo(present_depth);


        Mat match_result;
        if(ii>30) {
            match_result = new_folder.match_contours2(previous, previous_color, previous_depth, present, present_color,
                                                      present_depth);
            bool tpp=new_folder.if_stop;
            std::cout<<"if_stop    "<<tpp<<endl;
              if(ii%2==0)
            {
                imwrite("../data/color_draw/"+to_string(ii/2)+".png",new_folder.color_draw);
                imwrite("../data/depth_draw/"+to_string(ii/2)+".png",new_folder.depth_draw);
                imwrite("../data/match_result/"+to_string(ii/2)+".png",match_result);
            }
            imshow(depth_win,depth1_tp);
            imshow(color_win,new_folder.color_draw);
            imshow("result",new_folder.depth_draw);
            imshow(top_win,top_view);
            imshow("match", match_result);
            waitKey(100);
        }

        top_view.copyTo(previous);
        color_image.copyTo(previous_color);
        depth2_tp.copyTo(previous_depth);



        ii++;
//        if(ii%2==0)
//        {
//            imwrite("../data/color/"+to_string(ii/5)+".png",color_image);
//            imwrite("../data/top_view/"+to_string(ii/5)+".png",top_view);
//            imwrite("../data/depth/"+to_string(ii/5)+".png",depth2_tp);
//        }
    }
    return 0;
}