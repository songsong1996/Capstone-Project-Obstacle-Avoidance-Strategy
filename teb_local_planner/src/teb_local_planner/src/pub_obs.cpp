#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <stdlib.h>
//使用命名空间，省的每次都需要ros::
using namespace ros;
//这里使用cout进行打印输出
using namespace std;

class point2f
{
public:
    double x;
    double y;
double twist;
    point2f(double t1, double t2,double twis)
    {
        x = t1;
        y = t2;
        twist=twis;
    }

};

vector<point2f> Obs_list;

void init_Obs()
{
    for (int i = 0; i < 13; i++)
    {
        double random = rand() % 100 > 50 ? 1.0:-1.0;
        Obs_list.push_back(point2f(2.0 + 1.5 * i, (20 + rand() % 560) / 100,random*0.02));
    }
    for (int i = 0; i < 13; i++)
    {
        double random = rand() % 100 > 50 ? 1.0: -1.0;
        Obs_list.push_back(point2f(2.0 + 1.5 * i, (20 + rand() % 560) / 100,random*0.02));
    }
}

void pub_Obs(point2f &point, Publisher pub, int mode)
{
    tf::TransformBroadcaster odom_broadcaster;
    ros::Time current_time;
    current_time = ros::Time::now();
    if (mode == 1)
    {
        int flag =0;
        if (point.x + point.twist > 22 && point.x <= 22)
            flag = 2;
        if (point.x + point.twist < 2 && point.x >= 2)
            flag = 1;
        
        if (flag == 2)
        {
            point.twist= -0.02;
            point.x += point.twist;
        }
        else if (flag == 1)
        {
            point.twist = 0.02;
            point.x += point.twist;
        }
        else
        {
            point.x += point.twist;
        }
   
    }
    else
    {
         int flag = 0;
        if (point.y + point.twist > 5.8 && point.y <= 5.8)
            flag = 2;
        if (point.y+ point.twist < 0.2 && point.y >= 0.2)
            flag = 1;
        
        if (flag == 2)
        {
            point.twist = -0.02;
            point.y+= point.twist;
        }
        else if (flag == 1)
        {
            point.twist= 0.02;
            point.y += point.twist;
        }
        else
        {
             point.y += point.twist;
        }
     
    }

    double x = point.x;
    double y = point.y;
    double th = 0.0;
    double vx = 0.1;
    double vy = -0.1;
    double vth = 0.1;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    //publish the message
    pub.publish(odom);
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "obs_pub");
    ros::NodeHandle n;
    vector<Publisher> pub_list;
    init_Obs();
    for (int i = 0; i < 26; i++)
    {
        string topic = "robot_" + std::to_string(i + 1) + "/base_pose_ground_truth";
        Publisher pub = n.advertise<nav_msgs::Odometry>(topic, 1000);
        pub_list.push_back(pub);
    }

    Rate loop_rate(10);
    while (n.ok())
    {
        for (int i = 0; i < 13; i++)
        {
            pub_Obs(Obs_list[i], pub_list[i], 1);
        }
        for (int i = 13; i < 26; i++)
        {
            pub_Obs(Obs_list[i], pub_list[i], 2);
        }
        ros::spinOnce(); // check for incoming messages
        loop_rate.sleep();
    }
}
