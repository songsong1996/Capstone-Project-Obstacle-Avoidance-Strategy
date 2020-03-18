#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv::imshow("view",cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_8UC1)->image);
    cv::waitKey(20);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'mono8'.", msg->encoding.c_str());
  }
}

void color_Callback(const sensor_msgs::ImageConstPtr& color_msg)
{
    //cv_bridge::CvImagePtr color_ptr;
    try
    {
        cv::imshow("color_view", cv_bridge::toCvShare(color_msg, sensor_msgs::image_encodings::BGR8)->image);
        cv::waitKey(20); // 不断刷新图像，频率时间为int delay，单位为ms
    }
    catch (cv_bridge::Exception& e )
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", color_msg->encoding.c_str());
    }

}


void depth_Callback(const sensor_msgs::ImageConstPtr& depth_msg)
{
    //cv_bridge::CvImagePtr depth_ptr;
    try
    {
        cv::imshow("depth_view", cv_bridge::toCvShare(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1)->image);
        cv::waitKey(20);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'mono16'.", depth_msg->encoding.c_str());
    }

}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  cv::namedWindow("view");
  cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub1 = it.subscribe("camera/top_view_image", 2, imageCallback);
  image_transport::Subscriber sub2 = it.subscribe("camera/depth_result", 2, depth_Callback);
  image_transport::Subscriber sub3 = it.subscribe("camera/color_result", 2, color_Callback);
  image_transport::Subscriber sub4 = it.subscribe("camera/match_result", 2, color_Callback);

  while(ros::ok())
  {
       ros::spinOnce();
  }
  cv::destroyWindow("topview");
  cv::destroyWindow("depth");

  return 0;
}
