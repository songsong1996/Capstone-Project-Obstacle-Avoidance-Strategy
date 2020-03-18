//
// Created by songsong on 19-5-22.
//

//
// Created by songsong on 19-4-22.
//

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>



#include<iostream>

#include<string>
#include"camera_opencv/align_depth_color.h"
#include"camera_opencv/contours_processor.h"

using namespace std;
//namespace enc = sensor_msgs::image_encodings;


// 全局变量：图像矩阵和点云
cv_bridge::CvImagePtr color_ptr, depth_ptr;
cv::Mat color_pic, depth_pic;

void color_Callback(const sensor_msgs::ImageConstPtr& color_msg)
{
    //cv_bridge::CvImagePtr color_ptr;
    try
    {
        cv::imshow("color", cv_bridge::toCvShare(color_msg, sensor_msgs::image_encodings::BGR8)->image);
        color_ptr = cv_bridge::toCvCopy(color_msg, sensor_msgs::image_encodings::BGR8);

        cv::waitKey(20); // 不断刷新图像，频率时间为int delay，单位为ms
    }
    catch (cv_bridge::Exception& e )
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", color_msg->encoding.c_str());
    }
    color_pic = color_ptr->image;

    // output some info about the rgb image in cv format

//    //    cout<<"output some info about the rgb image in cv format"<<endl;
//    cout<<"rows of the rgb image = "<<color_pic.rows<<endl;
//    cout<<"cols of the rgb image = "<<color_pic.cols<<endl;
//    cout<<"type of rgb_pic's element = "<<color_pic.type()<<endl;
}


void depth_Callback(const sensor_msgs::ImageConstPtr& depth_msg)
{
    //cv_bridge::CvImagePtr depth_ptr;
    try
    {
        cv::imshow("depth", cv_bridge::toCvShare(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1)->image);
        depth_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);

//        cv::imshow("depth_view", cv_bridge::toCvShare(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1)->image);
//        depth_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);

        cv::waitKey(20);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'mono16'.", depth_msg->encoding.c_str());
    }

    depth_pic = depth_ptr->image;

//    // output some info about the depth image in cv format
//    cout<<"output some info about the depth image in cv format"<<endl;
//    cout<<"rows of the depth image = "<<depth_pic.rows<<endl;
//    cout<<"cols of the depth image = "<<depth_pic.cols<<endl;
//    cout<<"type of depth_pic's element = "<<depth_pic.type()<<endl;
    cout<<depth_pic.at<ushort>(240,320)<<endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "get_top_view");
    ros::NodeHandle nh;
    cv::namedWindow("color");
    cv::namedWindow("depth");
    //  cv::namedWindow("top");
    cv::startWindowThread();
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("camera/color/image_raw", 3, color_Callback);
    image_transport::Subscriber sub1 = it.subscribe("camera/aligned_depth_to_color/image_raw", 3, depth_Callback);

    image_transport::Publisher pub = it.advertise("camera/top_view_image", 1);

    image_transport::Publisher pub1 = it.advertise("camera/color_result", 1);
    image_transport::Publisher pub2 = it.advertise("camera/depth_result", 1);
   // image_transport::Publisher pub3 = it.advertise("camera/match_result", 1);

    double sample_rate = 20.0; // 1HZ，1秒发1次
    ros::Rate naptime(sample_rate); // use to regulate loop rate


    cout<<"depth value of depth map : "<<endl;

    int ii=0;
    Mat previous_depth,previous_color,present_depth,present_color,previous,present;
    match new_folder=match();
    if(ros::ok())
    {
        cv::Mat result_color=cv::Mat::zeros(color_pic.rows,color_pic.cols,CV_8UC3);
        cv::Mat result_depth=cv::Mat::zeros(depth_pic.rows,depth_pic.cols,CV_16UC1);

        // 遍历深度图
        for (int m = 0; m < depth_pic.rows; m++) {
            for (int n = 0; n < depth_pic.cols; n++) {

                // 获取深度图中(m,n)处的值
                //float d = depth_pic.ptr<float>(m)[n];//ushort d = depth_pic.ptr<ushort>(m)[n];
//                uint16_t d = depth_pic.ptr<uint16_t>(m)[n];//ushort d = depth_pic.ptr<ushort>(m)[n];
                ushort d=depth_pic.at<ushort>(m,n);


                // d 可能没有值，若如此，跳过此点

                    // d 存在值，则向点云增加一个点
                 if (d <= 800)
                {
                    result_depth.at<ushort>(m,n)=d;

                } else
                {
                    result_depth.at<ushort>(m,n)=0;
                }

                // 从rgb图像中获取它的颜色
                // 从rgb图像中获取它的颜色
                // rgb是三通道的BGR格式图，所以按下面的顺序获取颜色

                result_color.at<cv::Vec3b>(m,n)[0]=color_pic.at<cv::Vec3b>(m,n)[0];
                result_color.at<cv::Vec3b>(m,n)[1] = color_pic.at<cv::Vec3b>(m,n)[1];
                result_color.at<cv::Vec3b>(m,n)[2] = color_pic.at<cv::Vec3b>(m,n)[2];
            }
        }

//        cv::imshow("depth_result",result_depth);
//        cv::waitKey(20);

        result_depth=SmoothImage(result_depth);
        cv::Mat top_view=depth_top(result_depth,600,result_depth.cols,0,result_depth.rows);
//        cv::imshow("top_view",top_view);

//        sensor_msgs::ImagePtr msg1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", result_color).toImageMsg();
//        pub1.publish(msg1);
        sensor_msgs::ImagePtr msg2 = cv_bridge::CvImage(std_msgs::Header(), "mono16", result_depth).toImageMsg();
        pub2.publish(msg2);
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", top_view).toImageMsg();
        pub.publish(msg);


        // cv::waitKey(20);
//        ros::spinOnce(); //allow data update from callback;
//        naptime.sleep(); // wait for remainder of specified period;
    }

    while (ros::ok())
    {
        ii++;

        cv::Mat result_color=cv::Mat::zeros(color_pic.rows,color_pic.cols,CV_8UC3);
        cv::Mat result_depth=cv::Mat::zeros(depth_pic.rows,depth_pic.cols,CV_16UC1);

        // 遍历深度图
        for (int m = 0; m < depth_pic.rows; m++) {
            for (int n = 0; n < depth_pic.cols; n++) {

                // 获取深度图中(m,n)处的值
                //float d = depth_pic.ptr<float>(m)[n];//ushort d = depth_pic.ptr<ushort>(m)[n];
//                uint16_t d = depth_pic.ptr<uint16_t>(m)[n];//ushort d = depth_pic.ptr<ushort>(m)[n];
                ushort d=depth_pic.at<ushort>(m,n);


                // d 可能没有值，若如此，跳过此点
                    // d 存在值，则向点云增加一个点
                 if (d <= 800)
                {
                    result_depth.at<ushort>(m,n)=d;

                } else
                {
                    result_depth.at<ushort>(m,n)=0;
                }

                // 从rgb图像中获取它的颜色
                // 从rgb图像中获取它的颜色
                // rgb是三通道的BGR格式图，所以按下面的顺序获取颜色

                result_color.at<cv::Vec3b>(m,n)[0]=color_pic.at<cv::Vec3b>(m,n)[0];
                result_color.at<cv::Vec3b>(m,n)[1] = color_pic.at<cv::Vec3b>(m,n)[1];
                result_color.at<cv::Vec3b>(m,n)[2] = color_pic.at<cv::Vec3b>(m,n)[2];
            }
        }

//        cv::imshow("depth_result",result_depth);
//        cv::waitKey(20);

         result_depth=SmoothImage(result_depth);
        cv::Mat top_view=depth_top(result_depth,600,result_depth.cols,0,result_depth.rows);
//        cv::imshow("top_view",top_view);

        Mat result_depth_tp=stressDepth(result_depth);
        result_depth_tp.copyTo( present_depth);
        result_color.copyTo( present_color);
        top_view.copyTo(present);
        Mat match_image;
        if(ii==60)
        {
            Mat result_depth_tp=stressDepth(result_depth);
            result_depth_tp.copyTo( previous_depth);
            result_color.copyTo( previous_color);
            top_view.copyTo(previous);
        }
        if(ii>60)
        {
            match_image=new_folder.match_contours2(previous,previous_color,previous_depth,present,present_color,present_depth);
            cv::imshow("match_result",match_image);
            cv::waitKey(20);
//            sensor_msgs::ImagePtr msg3 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", match_image).toImageMsg();
//            pub3.publish(msg3);
        }
        result_depth_tp.copyTo( previous_depth);
        result_color.copyTo( previous_color);
        top_view.copyTo(previous);

        sensor_msgs::ImagePtr msg1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", result_color).toImageMsg();
        pub1.publish(msg1);
        sensor_msgs::ImagePtr msg2 = cv_bridge::CvImage(std_msgs::Header(), "mono16", result_depth_tp).toImageMsg();
        pub2.publish(msg2);
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", top_view).toImageMsg();
        pub.publish(msg);

        // cv::waitKey(20);
        ros::spinOnce(); //allow data update from callback;
        naptime.sleep(); // wait for remainder of specified period;
    }
    cv::destroyWindow("color_view");
    cv::destroyWindow("depth_view");
    //  cv::destroyWindow("top_view");
    return 0;
}
