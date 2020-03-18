#include <iostream>
using namespace std;

#include <sstream>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <cstring>

#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>


using namespace cv;

#include<librealsense2/rs.hpp>
#include<librealsense2/rsutil.h>
#include"camera_opencv/align_depth_color.h"
//获取深度像素对应长度单位（米）的换算比例
float get_depth_scale(rs2::device dev)
{
    // Go over the device's sensors
    for (rs2::sensor& sensor : dev.query_sensors())
    {
        // Check if the sensor if a depth sensor
        if (rs2::depth_sensor dpt = sensor.as<rs2::depth_sensor>())
        {
            return dpt.get_depth_scale();
        }
    }
    throw std::runtime_error("Device does not have a depth sensor");
}
//深度图对齐到彩色图函数
Mat align_Depth2Color(Mat depth,Mat color,rs2::pipeline_profile profile){
    //声明数据流
    auto depth_stream=profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
    auto color_stream=profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();

    //获取内参
    const auto intrinDepth=depth_stream.get_intrinsics();
    const auto intrinColor=color_stream.get_intrinsics();

    //直接获取从深度摄像头坐标系到彩色摄像头坐标系的欧式变换矩阵
    //auto  extrinDepth2Color=depth_stream.get_extrinsics_to(color_stream);
    rs2_extrinsics  extrinDepth2Color;
    rs2_error *error;
    rs2_get_extrinsics(depth_stream,color_stream,&extrinDepth2Color,&error);

    //平面点定义
    float pd_uv[2],pc_uv[2];
    //空间点定义
    float Pdc3[3],Pcc3[3];

    //获取深度像素与现实单位比例（D435默认1毫米）
    float depth_scale = get_depth_scale(profile.get_device());
//    uint16_t depth_max=0;
//    for(int row=0;row<depth.rows;row++){
//        for(int col=0;col<depth.cols;col++){
//            if(depth_max<depth.at<uint16_t>(row,col))
//                depth_max=depth.at<uint16_t>(row,col);
//        }
//    }
    int y=0,x=0;
    //初始化结果
    Mat result=Mat::zeros(color.rows,color.cols,CV_8UC3);
    Mat depth_align=Mat::zeros(color.rows,color.cols,CV_16U);

    //对深度图像遍历
    for(int row=0;row<depth.rows;row++){
        for(int col=0;col<depth.cols;col++){
            //将当前的(x,y)放入数组pd_uv，表示当前深度图的点
            pd_uv[0]=col;
            pd_uv[1]=row;
            //取当前点对应的深度值
            uint16_t depth_value=depth.at<uint16_t>(row,col);
            //换算到米
            float depth_m=depth_value*depth_scale;
            //将深度图的像素点根据内参转换到深度摄像头坐标系下的三维点
            rs2_deproject_pixel_to_point(Pdc3,&intrinDepth,pd_uv,depth_m);
            //将深度摄像头坐标系的三维点转化到彩色摄像头坐标系下
            rs2_transform_point_to_point(Pcc3,&extrinDepth2Color,Pdc3);
            //将彩色摄像头坐标系下的深度三维点映射到二维平面上
            rs2_project_point_to_pixel(pc_uv,&intrinColor,Pcc3);

            //取得映射后的（u,v)
            x=(int)pc_uv[0];
            y=(int)pc_uv[1];
//            if(x<0||x>color.cols)
//                continue;
//            if(y<0||y>color.rows)
//                continue;
            //最值限定
            x=x<0? 0:x;
            x=x>depth.cols-1 ? depth.cols-1:x;
            y=y<0? 0:y;
            y=y>depth.rows-1 ? depth.rows-1:y;

//            //将成功映射的点用彩色图对应点的RGB数据覆盖
//            for(int k=0;k<3;k++){
//                //这里设置了只显示1米距离内的东西
//                if(depth_m<1)
//                    result.at<cv::Vec3b>(y,x)[k]=
//                            color.at<cv::Vec3b>(y,x)[k];
//            }
            if(depth_m<0.6)
                depth_align.at<uint16_t>(y,x)=depth.at<uint16_t>(row,col);

//            depth_align.at<uint16_t>(y,x)=depth.at<uint16_t>(row,col);

        }
    }
    cout<<depth_align.at<uint16_t>(240,320)<<" "<<depth.at<uint16_t>(240,320)<<endl;
//    cv::namedWindow("overlap",WINDOW_AUTOSIZE);
//    cv::namedWindow("aligndepth",WINDOW_AUTOSIZE);
//
//    cv::imshow("overlap",result);
//    cv::imshow("aligndepth",depth_align);
//    cv:waitKey(0);

    // return result;
    return depth_align;

}


Mat depth_top(Mat depth_align,int rows,int cols,int  height_min,int height_max)
{
    Mat top_view=Mat::zeros(rows,cols,CV_8UC1);
//    for(int i=0;i<rows;i++)
//        for(int j=0;j<cols;j++)
//            top_view.at<uchar>(i,j)=255;

    for(int i=height_min;i<height_max;i++)
        for(int j=0;j<depth_align.cols;j++)
            if(depth_align.at<uint16_t>(i,j)!=0 && depth_align.at<uint16_t>(i,j)<500)
                top_view.at<uchar>((int)(rows-depth_align.at<uint16_t >(i,j)),j)=255;
    return top_view;
}



Mat SmoothImage(Mat image)
{
    Mat result = Mat::zeros(image.size(), CV_16UC1);
//    char buf1[100];
//    char buf2[100];
    vector<Point2i> NeihborPos;
    NeihborPos.push_back(Point2i(-1, 0));
    NeihborPos.push_back(Point2i(1, 0));
    NeihborPos.push_back(Point2i(0, -1));
    NeihborPos.push_back(Point2i(0, 1));
    NeihborPos.push_back(Point2i(-1, -1));
    NeihborPos.push_back(Point2i(-1, 1));
    NeihborPos.push_back(Point2i(1, -1));
    NeihborPos.push_back(Point2i(1, 1));
    int Neihbor_count=8;
    int CurrX=0,CurrY=0;
//
//        sprintf(buf1,"/home/yangjunfeng/workspace_lj/workspace/clion_project/disp_new/disp_%d.png",i);
//        sprintf(buf2,"/home/yangjunfeng/workspace_lj/workspace/clion_project/disp_new2/disp_%d.png",i);


        for(int i = 0; i < image.rows; ++i)
        {
            int bad_c_count=0;
            vector<Point2i> temp_local;
            uint16_t* s_data = image.ptr<uint16_t>(i);
            uint16_t* d_data = result.ptr<uint16_t>(i);
            for(int j = 0; j < image.cols; ++j)
            {
                //temp_local.push_back(Point2i(j,i));
                if(s_data[j] != 0) d_data[j]=s_data[j];
                else{
                    temp_local.push_back(Point2i(j,i));
                    bad_c_count++;
                    int pixel_sum=0;
                    int nerbor_suit=0;
                    for(int k=0;k<Neihbor_count;++k){
                        CurrX=temp_local.at(bad_c_count-1).x+NeihborPos.at(k).x;
                        CurrY=temp_local.at(bad_c_count-1).y+NeihborPos.at(k).y;
                        if (CurrX>=0&&CurrX<image.cols&&CurrY>=0&&CurrY<image.rows&&image.at<uint16_t>(CurrY, CurrX)!=0){
                            pixel_sum+=image.at<uint16_t>(CurrY, CurrX);
                            nerbor_suit++;
                        }

                    }
                    //cout<<"pixel_sum:"<<pixel_sum<<endl<<"nerbor_suit:"<<nerbor_suit<<endl;
                    //float kkk=pixel_sum/nerbor_suit;
                    //cout<<kkk<<endl;
                    if (nerbor_suit!=0)
                        d_data[j]=int(pixel_sum/nerbor_suit);
                    //d_data[j]=255;
                }

            }
        }
//        namedWindow("tp",WINDOW_NORMAL);
//        imshow("tp",result);
//        waitKey(10);
//        imwrite(buf2,dst);

return result;

}


Mat stressDepth(Mat &image)
{
    Mat result = Mat::zeros(image.rows, image.cols, CV_16UC1);
    for (int i = 0; i < image.rows; i++)
        for (int j = 0; j < image.cols; j++) {
            if(image.at<uint16_t>(i,j)>600)result.at<uint16_t >(i,j)=0;
            result.at<uint16_t>(i, j) = image.at<uint16_t>(i, j) *100.0;
        }
    return result;
}
