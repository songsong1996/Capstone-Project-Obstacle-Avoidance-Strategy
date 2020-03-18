
#include <teb_local_planner/teb_local_planner_ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/Marker.h>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <stdio.h>

using namespace teb_local_planner; // it is ok here to import everything for testing purposes

using namespace cv;
using namespace std;
//============== Class Definations ===============

// ============= Global Variables ================
// Ok global variables are bad, but here we only have a simple testing node.

std::vector<Point2f> obs_list;
std::vector<Point2f> robot_position_list;
//std::vector<ObstaclePtr> obst_vector;
Point2f start_point, goal_point, robot_position;
//TebConfig config;

// boost::shared_ptr<dynamic_reconfigure::Server<TebLocalPlannerReconfigureConfig>> dynamic_recfg;
// ros::Subscriber obs_sub;
ros::Subscriber robot_odom, simple_goal;
std::vector<ros::Subscriber> obs_vel_subs;
unsigned int no_fixed_obstacles;

double scale = 30;
int rec_width = 25;
int rec_height = 6;
int all_width = 28;
int all_height = 8;
int rows = all_height * scale;
int cols = all_width * scale;
int alll = 0;
// =========== Function declarations =============
void CB_obstacle_point(const nav_msgs::Odometry::ConstPtr &msg, const unsigned int id);
void CB_RobotPosition(const nav_msgs::Odometry::ConstPtr &msg);
void CB_getGoals(const geometry_msgs::PoseStamped::ConstPtr &msg);
//draw
Point2f Coor_Zoom(double x, double y);
void Draw_Obs(Mat &back_image, std::vector<Point2f> &obs_list, Point2f &robot);
Mat Draw(std::vector<Point2f> &obs_list, Point2f &robot);
Mat init_Back_Image();

// =============== Main function =================
int main(int argc, char **argv)
{

  ros::init(argc, argv, "draw_subs");
  ros::NodeHandle n;
  for (int i = 0; i < 13; i++)
    obs_list.push_back(Point2f(0, 0));
 // Mat back1 = init_Back_Image();
  robot_odom = n.subscribe("/robot_0/base_pose_ground_truth", 10, CB_RobotPosition);
  simple_goal = n.subscribe("/robot_0/move_base_simple/goal", 10, CB_getGoals);

  for (unsigned int i = 0; i < 13; ++i)
  {
    // setup callbacks for setting obstacle velocities
    std::string topic = "robot_" + std::to_string(i + 1) + "/base_pose_ground_truth";
    //obst_vel_subs.push_back(n.subscribe<geometry_msgs::Twist>(topic, 1, boost::bind(&CB_setObstacleVelocity, _1, i)));
    obs_vel_subs.push_back(n.subscribe<nav_msgs::Odometry>(topic, 10, boost::bind(&CB_obstacle_point, _1, i + 1)));
  }

  // for(int i=0;i<10;i++)
  // cout<<i<<endl;
  while (ros::ok())
  {
    Mat back1 = Draw(obs_list, robot_position);
    imshow("tp", back1);
    waitKey(10);
    // cv::waitKey(20);
    ros::spinOnce(); //allow data update from callback;
  }
  //ros::spin();
  return 0;
}

void CB_obstacle_point(const nav_msgs::Odometry::ConstPtr &msg, const unsigned int id)
{
  geometry_msgs::Quaternion orientation = msg->pose.pose.orientation;
  tf::Matrix3x3 mat(tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));
  double yaw, pitch, roll;
  mat.getEulerYPR(yaw, pitch, roll);
  // std::cout << yaw << std::endl;
  //Get the matrix represented as euler angles around YXZ, roundtrip with setEulerYPR.
  //Yaw around Z axis; Pitch around Y axis; Roll around X axis
  obs_list[id - 1].x = msg->pose.pose.position.x;
  obs_list[id - 1].y = msg->pose.pose.position.y;
  //PointObstacle tp(msg->pose.pose.position.x,msg->pose.pose.position.y);
  // obst_vector.push_back(boost::make_shared<PointObstacle>(msg->pose.pose.position.x, msg->pose.pose.position.y));
}

void CB_RobotPosition(const nav_msgs::Odometry::ConstPtr &msg)
{
  geometry_msgs::Quaternion orientation = msg->pose.pose.orientation;
  tf::Matrix3x3 mat(tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));
  double yaw, pitch, roll;
  mat.getEulerYPR(yaw, pitch, roll);
  // std::cout << yaw << std::endl;
  //Get the matrix represented as euler angles around YXZ, roundtrip with setEulerYPR.
  //Yaw around Z axis; Pitch around Y axis; Roll around X axis
  robot_position.x = msg->pose.pose.position.x;
  robot_position.y = msg->pose.pose.position.y;
  robot_position_list.push_back(robot_position);
}

void CB_getGoals(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  goal_point.x = msg->pose.position.x;
  goal_point.y = msg->pose.position.y;
}

//draw
Point2f Coor_Zoom(double x, double y)
{
  double xx = ((all_width - rec_width) / 2.0 + x) * scale;
  double yy = ((all_height - rec_height) / 2.0 + (rec_height - y)) * scale;
  return cv::Point2f(xx, yy);
}

void Draw_Obs(Mat &back_image, std::vector<Point2f> &obs_list, Point2f &robot)
{
  for (int i = 0; i < obs_list.size(); i++)
  {
    Point2f trans = Coor_Zoom(obs_list[i].x, obs_list[i].y);
    double obs_size = 8;
    Point2f tp1(trans.x - obs_size, trans.y - obs_size), tp2(trans.x + obs_size, trans.y + obs_size);
    rectangle(back_image, tp1, tp2, Scalar(255, 0, 0), 2);
  }

  Point2f trans = Coor_Zoom(robot.x, robot.y);
  double ro_size = 8;
  Point2f tp1(trans.x - ro_size, trans.y - ro_size), tp2(trans.x + ro_size, trans.y + ro_size);
  RotatedRect ro(trans, Size2f(ro_size * 2, ro_size * 2), 0);
  ellipse(back_image, ro, Scalar(0, 0, 255), 2);
}

Mat Draw(std::vector<Point2f> &obs_list, Point2f &robot)
{
  Mat background = Mat::ones(rows, cols, CV_8UC3);
  for (int i = 0; i < rows; i++)
    for (int j = 0; j < cols; j++)
      for (int k = 0; k < 3; k++)
        background.at<Vec3b>(i, j)[k] = 255;
  Point2f tp1(Coor_Zoom(0, 0)), tp2(Coor_Zoom(25, 6));
  tp1.x -= 8;
  tp1.y += 8;
  tp2.x += 8;
  tp2.y -= 8;
  rectangle(background, tp1, tp2, Scalar(0, 0, 0), 2);
  Draw_Obs(background, obs_list, robot);
  for (int i = 0; i < obs_list.size() - 1; i++)
    line(background, obs_list[i], obs_list[i + 1], Scalar(0, 0, 255));
  return background;
}

Mat init_Back_Image()
{
  Mat background = Mat::ones(rows, cols, CV_8UC3);
  for (int i = 0; i < rows; i++)
    for (int j = 0; j < cols; j++)
      for (int k = 0; k < 3; k++)
        background.at<Vec3b>(i, j)[k] = 255;
  Point2f tp1(Coor_Zoom(0, 0)), tp2(Coor_Zoom(25, 6));
  tp1.x -= 8;
  tp1.y += 8;
  tp2.x += 8;
  tp2.y -= 8;
  rectangle(background, tp1, tp2, Scalar(0, 0, 0), 2);
  return background;
}
