
#include <teb_local_planner/teb_local_planner_ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/Marker.h>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <teb_local_planner/pose_se2.h>
#include <stdio.h>
#include <math.h>
#include "kalman/kalman.h"

using namespace teb_local_planner; // it is ok here to import everything for testing purposes
using namespace cv;
using namespace std;
//============== Class Definations ===============
class Obs
{
public:
  Point2f point_now;
  Point2d point_predict;
  kalman_filter kf;
  Obs(){};
  Obs(Point2f p1)
  {
    point_now.x = p1.x;
    point_now.y = p1.y;
    Point tp(point_now.x, point_now.y);
    kf = kalman_filter(tp);
  }
};

// ============= Global Variables ================
// Ok global variables are bad, but here we only have a simple testing node.
double const PI = 3.1415926535898;
HomotopyClassPlanner planner, planner_local;
TebVisualizationPtr visual;
std::vector<ObstaclePtr> obst_vector;
std::vector<ObstaclePtr> obst_vector_local;
ViaPointContainer via_points;
TebConfig config;
boost::shared_ptr<dynamic_reconfigure::Server<TebLocalPlannerReconfigureConfig>> dynamic_recfg;
ros::Subscriber custom_obst_sub;
ros::Subscriber via_points_sub;
ros::Subscriber clicked_points_sub;
RobotFootprintModelPtr robot_model;

std::vector<ros::Subscriber> obs_vel_subs;
unsigned int no_fixed_obstacles;
ros::Subscriber robot_odom, simple_goal;
std::vector<Point2f> obs_list, obs_list_local;
std::vector<Obs> obs_global;

std::vector<Point2f> robot_position_list;
std::vector<VertexPose *> robot_plan_list;
vector<geometry_msgs::PoseStamped> robot_plan_message;
Point2f start_point, goal_point, robot_position, robot_position_plan;
PoseSE2 robot_current_pose, robot_current_pose_plan, robot_current_pose0;
double scale = 30;
int rec_width = 25;
int rec_height = 6;
int all_width = 28;
int all_height = 8;
int rows = all_height * scale;
int cols = all_width * scale;
//double velocity=0.025;
double vt = 30;

Mat relative_im = Mat::ones(200, 200, CV_8UC3);

// =========== Function declarations =============
void CB_mainCycle(const ros::TimerEvent &e);
void CB_publishCycle(const ros::TimerEvent &e);
void CB_reconfigure(TebLocalPlannerReconfigureConfig &reconfig, uint32_t level);
void CB_customObstacle(const costmap_converter::ObstacleArrayMsg::ConstPtr &obst_msg);
void CreateInteractiveMarker(const double &init_x, const double &init_y, unsigned int id, std::string frame, interactive_markers::InteractiveMarkerServer *marker_server, interactive_markers::InteractiveMarkerServer::FeedbackCallback feedback_cb);
void CB_obstacle_marker(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
void CB_setObstacleVelocity(const geometry_msgs::TwistConstPtr &twist_msg, const unsigned int id);

void CB_obstacle_point(const nav_msgs::Odometry::ConstPtr &msg, const unsigned int id);
void CB_RobotPosition(const nav_msgs::Odometry::ConstPtr &msg);
void CB_getGoals(const geometry_msgs::PoseStamped::ConstPtr &msg);

double cal_dist(Point2f &p1, Point2f &p2);
Point2f relative_pose(PoseSE2 robot, Point2f obs_position);
Point predict_min_dist(Obs &obs, int &id, double &dist_min);
//void publishLocalPlanAndPoses(TimedElasticBand &teb, TebConfig &config);

//my local plan
void init_Pose_list(vector<PoseSE2> global_plan, Point2f &start, Point2f &end);
std::vector<geometry_msgs::PoseStamped> global_plan_message(Point2f &start, Point2f &end);

//draw
Point2f Coor_Zoom(double x, double y);
void Draw_Obs(Mat &back_image, std::vector<Point2f> &obs_list, PoseSE2 &robot_current_pose);
Mat Draw(std::vector<Point2f> &obs_list, PoseSE2 &robot_current_pose);
Mat init_Back_Image();
Mat Draw_Arc(Mat back, Point2f robot, double yaw, double num);
void draw_Poses(PoseSequence &poses, Mat &background);
void draw_Poses(TimedElasticBand &teb, Mat &background);
int count_num = 0;

// =============== Main function =================
int main(int argc, char **argv)
{
  for (int i = 0; i < relative_im.rows; i++)
    for (int j = 0; j < relative_im.cols; j++)
      for (int k = 0; k < 3; k++)
        relative_im.at<Vec3b>(i, j)[k] = 255;
  ros::init(argc, argv, "test");
  ros::NodeHandle n;
  // load ros parameters from node handle
  config.loadRosParamFromNodeHandle(n);
 // cout<<"tppp"<<endl;

  for (int i = 0; i < 13; i++)
  {
    obs_list.push_back(Point2f(0, 0));
    obst_vector.push_back(boost::make_shared<PointObstacle>(0, 0));
    Obs tp(Point2f(0, 0));
    obs_global.push_back(tp);
  }

  // Mat back1 = init_Back_Image();
  robot_odom = n.subscribe("/robot_0/base_pose_ground_truth", 10, CB_RobotPosition);
  simple_goal = n.subscribe("/robot_0/move_base_simple/goal", 10, CB_getGoals);

  for (unsigned int i = 0; i < 13; ++i)
  {
    // setup callbacks for setting obstacle velocities
    std::string topic = "robot_" + std::to_string(i + 1) + "/base_pose_ground_truth";
    obs_vel_subs.push_back(n.subscribe<nav_msgs::Odometry>(topic, 10, boost::bind(&CB_obstacle_point, _1, i + 1)));
  }

  // Setup visualization
  visual = TebVisualizationPtr(new TebVisualization(n, config));

  // Setup robot shape model
  robot_model = TebLocalPlannerROS::getRobotFootprintFromParamServer(n);
  // Setup planner (homotopy class planning or just the local teb planner)
  //if (config.hcp.enable_homotopy_class_planning)
  // planner = PlannerInterfacePtr(new HomotopyClassPlanner(config, &obst_vector, robot_model));
  planner = HomotopyClassPlanner(config, &obst_vector, robot_model, visual);
  planner_local = HomotopyClassPlanner(config, &obst_vector_local, robot_model, visual);
  robot_plan_message = global_plan_message(robot_position_plan, goal_point);
  //cout<<"robot_plan_message0   "<<robot_plan_message.size()<<endl;
  //init_Pose_list(robot_plan_list, robot_position, goal_point);
  ros::Timer cycle_timer = n.createTimer(ros::Duration(0.3), CB_mainCycle);
  ros::spin();
  //cout << "===========================" << endl;
  return 0;
}

// Planning loop
void CB_mainCycle(const ros::TimerEvent &e)
{
  planner_local = HomotopyClassPlanner(config, &obst_vector_local, robot_model, visual);
  cout << "obst_vector_local   " << obst_vector_local.size() << endl;
  //cout << "1" << endl;
  if (robot_plan_message.size() != 0)
  {
    planner_local.plan(robot_plan_message); // hardcoded start and goal for testing purposes
    cout << "2" << endl;
    TebOptimalPlannerPtr teb_optim_planner_ptr = planner_local.bestTeb();
    Mat back1 = Draw(obs_list, robot_current_pose_plan);
    Mat tp;
    back1.copyTo(tp);
    for (int i = 1; i <= 3; i++)
      tp = Draw_Arc(tp, robot_position_plan, -robot_current_pose_plan.theta(), i);
    tp.copyTo(back1);
    if (teb_optim_planner_ptr)
    {
      TimedElasticBand &teb_band = teb_optim_planner_ptr->teb();
      robot_current_pose_plan.x() = teb_band.Pose(1).x();
      robot_current_pose_plan.y() = teb_band.Pose(1).y();
      robot_current_pose_plan.theta() = teb_band.Pose(1).theta();


      Point2f tppppppp(robot_position_plan.x,robot_position_plan.y);

      robot_position_plan.x = robot_current_pose_plan.x();
      robot_position_plan.y = robot_current_pose_plan.y();

      cout<<"rodist="<<cal_dist(tppppppp,robot_position_plan)<<endl;

      robot_plan_message = global_plan_message(robot_position_plan, goal_point);
      draw_Poses(teb_band, back1);
    }
    imshow("tp1", relative_im);
    imshow("tp", back1);

    waitKey(10);
  }
  else
  {
    robot_plan_message = global_plan_message(robot_position_plan, goal_point);
    Mat back1 = Draw(obs_list, robot_current_pose_plan);
    Mat tp;
    back1.copyTo(tp);
    for (int i = 1; i <= 3; i++)
      tp = Draw_Arc(tp, robot_position_plan, -robot_current_pose_plan.theta(), i);
    tp.copyTo(back1);
    imshow("tp", back1);
    waitKey(10);
    // robot_position_plan.x = robot_plan_message[0].pose.position.x;
    // robot_position_plan.y = robot_plan_message[0].pose.position.y;
    // robot_current_pose_plan.x() = robot_plan_message[0].pose.position.x;
    // robot_current_pose_plan.y() = robot_plan_message[0].pose.position.y;
    // geometry_msgs::Quaternion orientation = robot_plan_message[0].pose.orientation;
    // tf::Matrix3x3 mat(tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));
    // double yaw, pitch, roll;
    // mat.getEulerYPR(yaw, pitch, roll);
    // robot_current_pose_plan.theta() = yaw;
    // //cout << robot_plan_message.size() << endl;
  }
}

// Visualization loop
void CB_publishCycle(const ros::TimerEvent &e)
{
  planner.visualize();

  visual->publishObstacles(obst_vector);
  visual->publishViaPoints(via_points);
}

void CB_reconfigure(TebLocalPlannerReconfigureConfig &reconfig, uint32_t level)
{
  config.reconfigure(reconfig);
}

void CreateInteractiveMarker(const double &init_x, const double &init_y, unsigned int id, std::string frame, interactive_markers::InteractiveMarkerServer *marker_server, interactive_markers::InteractiveMarkerServer::FeedbackCallback feedback_cb)
{
  // create an interactive marker for our server
  visualization_msgs::InteractiveMarker i_marker;
  i_marker.header.frame_id = frame;
  i_marker.header.stamp = ros::Time::now();
  std::ostringstream oss;
  //oss << "obstacle" << id;
  oss << id;
  i_marker.name = oss.str();
  i_marker.description = "Obstacle";
  i_marker.pose.position.x = init_x;
  i_marker.pose.position.y = init_y;
  i_marker.pose.orientation.w = 1.0f; // make quaternion normalized

  // create a grey box marker
  visualization_msgs::Marker box_marker;
  box_marker.type = visualization_msgs::Marker::CUBE;
  box_marker.id = id;
  box_marker.scale.x = 0.2;
  box_marker.scale.y = 0.2;
  box_marker.scale.z = 0.2;
  box_marker.color.r = 0.5;
  box_marker.color.g = 0.5;
  box_marker.color.b = 0.5;
  box_marker.color.a = 1.0;
  box_marker.pose.orientation.w = 1.0f; // make quaternion normalized

  // create a non-interactive control which contains the box
  visualization_msgs::InteractiveMarkerControl box_control;
  box_control.always_visible = true;
  box_control.markers.push_back(box_marker);

  // add the control to the interactive marker
  i_marker.controls.push_back(box_control);

  // create a control which will move the box, rviz will insert 2 arrows
  visualization_msgs::InteractiveMarkerControl move_control;
  move_control.name = "move_x";
  move_control.orientation.w = 0.707107f;
  move_control.orientation.x = 0;
  move_control.orientation.y = 0.707107f;
  move_control.orientation.z = 0;
  move_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;

  // add the control to the interactive marker
  i_marker.controls.push_back(move_control);

  // add the interactive marker to our collection
  marker_server->insert(i_marker);
  marker_server->setCallback(i_marker.name, feedback_cb);
}

void CB_obstacle_marker(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  std::stringstream ss(feedback->marker_name);
  unsigned int index;
  ss >> index;

  if (index >= no_fixed_obstacles)
    return;
  PointObstacle *pobst = static_cast<PointObstacle *>(obst_vector.at(index).get());
  pobst->position() = Eigen::Vector2d(feedback->pose.position.x, feedback->pose.position.y);
}

void CB_customObstacle(const costmap_converter::ObstacleArrayMsg::ConstPtr &obst_msg)
{
  // resize such that the vector contains only the fixed obstacles specified inside the main function
  obst_vector.resize(no_fixed_obstacles);

  // Add custom obstacles obtained via message (assume that all obstacles coordiantes are specified in the default planning frame)
  for (size_t i = 0; i < obst_msg->obstacles.size(); ++i)
  {
    if (obst_msg->obstacles.at(i).polygon.points.size() == 1)
    {
      obst_vector.push_back(ObstaclePtr(new PointObstacle(obst_msg->obstacles.at(i).polygon.points.front().x,
                                                          obst_msg->obstacles.at(i).polygon.points.front().y)));
    }
    else
    {
      PolygonObstacle *polyobst = new PolygonObstacle;
      for (size_t j = 0; j < obst_msg->obstacles.at(i).polygon.points.size(); ++j)
      {
        polyobst->pushBackVertex(obst_msg->obstacles.at(i).polygon.points[j].x,
                                 obst_msg->obstacles.at(i).polygon.points[j].y);
      }
      polyobst->finalizePolygon();
      obst_vector.push_back(ObstaclePtr(polyobst));
    }

    if (!obst_vector.empty())
      obst_vector.back()->setCentroidVelocity(obst_msg->obstacles.at(i).velocities, obst_msg->obstacles.at(i).orientation);
  }
}

void CB_setObstacleVelocity(const geometry_msgs::TwistConstPtr &twist_msg, const unsigned int id)
{
  if (id >= obst_vector.size())
  {
    ROS_WARN("Cannot set velocity: unknown obstacle id.");
    return;
  }

  Eigen::Vector2d vel(twist_msg->linear.x, twist_msg->linear.y);
  obst_vector.at(id)->setCentroidVelocity(vel);
}

void CB_obstacle_point(const nav_msgs::Odometry::ConstPtr &msg, const unsigned int id)
{
  cout << id << endl;
  geometry_msgs::Quaternion orientation = msg->pose.pose.orientation;
  tf::Matrix3x3 mat(tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));
  double yaw, pitch, roll;
  mat.getEulerYPR(yaw, pitch, roll);
  //std::cout << "yaw="<<yaw << std::endl;
  //Get the matrix represented as euler angles around YXZ, roundtrip with setEulerYPR.
  //Yaw around Z axis; Pitch around Y axis; Roll around X axis
  Point2f tp_point(msg->pose.pose.position.x, msg->pose.pose.position.y);
  // Point2f tpppppppp(obs_list[id - 1].x,obs_list[id - 1].y);
  // cout<<"obs_velo="<<cal_dist(tp_point, tpppppppp)<<endl;
  obs_list[id - 1].x = msg->pose.pose.position.x;
  obs_list[id - 1].y = msg->pose.pose.position.y;

  obs_global[id - 1].kf.update(Point(tp_point.x, tp_point.y));
  obs_global[id - 1].point_now = Point2f(tp_point.x, tp_point.y);

  obst_vector[id - 1] = boost::make_shared<PointObstacle>(msg->pose.pose.position.x, msg->pose.pose.position.y);
  //cout<<id<<endl;
  if (id == 1)
  {
    vector<ObstaclePtr> vtTemp;
    vtTemp.swap(obst_vector_local);
  }

  if (cal_dist(robot_position_plan, tp_point) < 1.0 * vt / scale * 1.0)
  {
    obst_vector_local.push_back(boost::make_shared<PointObstacle>(tp_point.x, tp_point.y));
  }
  else if (cal_dist(robot_position_plan, tp_point) < 2.0 * vt / scale * 1.0)
  {
    int idd = 1;
    double dist_min = 100;
    Point predict = predict_min_dist(obs_global[id - 1], idd, dist_min);
    if (dist_min < 1.0 * vt / scale * 1.0)
    {
      //if (idd < 10)
      obst_vector_local.push_back(boost::make_shared<PointObstacle>(predict.x, predict.y));
    }
  }
}

Point predict_min_dist(Obs &obs, int &id, double &dist_min)
{
  int flag = 0, num = 1;
  Point rela;
  while (flag < 2)
  {
    Point tp = obs.kf.predict_kt(num);
    Point2f p1(tp.x, tp.y);
    //circle(relative_im, Point(tp.x * 30 + 100, (100 - tp.y * 30)), 3, Scalar(0, 0, 255));
    Point2f p2(robot_plan_message[num].pose.position.x,robot_plan_message[num].pose.position.y);
    double dist = cal_dist(p1, p2);
    if (dist < dist_min)
    {
      dist_min = dist;
      id = num;
      rela.x = tp.x;
      rela.y = tp.y;
    }
    num++;
    if (dist < 1.0 * vt / scale * 1.0)
      flag = 2;
    if (dist > 2.0 * vt / scale * 1.0)
      flag = 2;
  }
  return rela;
}

void CB_RobotPosition(const nav_msgs::Odometry::ConstPtr &msg)
{
  geometry_msgs::Quaternion orientation = msg->pose.pose.orientation;
  tf::Matrix3x3 mat(tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));
  double yaw, pitch, roll;
  mat.getEulerYPR(yaw, pitch, roll);
  //std::cout << yaw << std::endl;
  //Get the matrix represented as euler angles around YXZ, roundtrip with setEulerYPR.
  //Yaw around Z axis; Pitch around Y axis; Roll around X axis
  robot_position.x = msg->pose.pose.position.x;
  robot_position.y = msg->pose.pose.position.y;
  //cout << robot_position.x << "  " << robot_position.y << endl;
  robot_position_list.push_back(robot_position);

  double sum_dist = 0;
  for (int i = 0; i < robot_position_list.size() - 1; i++)
  {
    sum_dist += cal_dist(robot_position_list[i], robot_position_list[i + 1]);
  }
  if (robot_position_list.size() > 1)
    //cout << sum_dist / (robot_position_list.size() - 1.0) << endl;
    //PoseSE2 tp_pose(msg->pose.pose.position.x,msg->pose.pose.position.y,yaw);
    robot_current_pose = PoseSE2(msg->pose.pose.position.x, msg->pose.pose.position.y, yaw);
  //robot_pose_list.push_back(tp_pose);
}

void CB_getGoals(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  vector<Point2f> vtTemp;
  vtTemp.swap(robot_position_list);
  goal_point.x = msg->pose.position.x;
  goal_point.y = msg->pose.position.y;
}

//draw
Point2f Coor_Zoom(double x, double y)
{
  double xx = ((all_width - rec_width) / 2.0 + x) * scale;
  double yy = ((all_height - rec_height) / 2.0 + (rec_height - y)) * scale;
  return Point2f(xx, yy);
}

void Draw_Obs(Mat &back_image, std::vector<Point2f> &obs_list, PoseSE2 &robot_current_pose)
{
  for (int i = 0; i < obs_list.size(); i++)
  {
    Point2f trans = Coor_Zoom(obs_list[i].x, obs_list[i].y);
    double obs_size = 4;
    // Point2f tp1(trans.x - obs_size, trans.y - obs_size), tp2(trans.x + obs_size, trans.y + obs_size);
    RotatedRect ro(trans, Size2f(obs_size * 2, obs_size * 2), 0);
    //rectangle(back_image, tp1, tp2, Scalar(255, 0, 0), 2);
    ellipse(back_image, ro, Scalar(0, 0, 255), 2);
  }
  Point2f trans = Coor_Zoom(robot_current_pose.x(), robot_current_pose.y());
  double ro_size = 4;
  //Point2f tp1(trans.x - ro_size, trans.y - ro_size), tp2(trans.x + ro_size, trans.y + ro_size);
  //rectangle(back_image, tp1, tp2, Scalar(255, 0, 0), 2);
  //cout << "robot_current_pose.theta()=" << robot_current_pose.theta() / PI * 180.0 << endl;
  RotatedRect ro(trans, Size2f(ro_size * 4, ro_size * 2), -robot_current_pose.theta() / PI * 180.0);
  Point2f vertices[4];
  ro.points(vertices);
  for (int i = 0; i < 4; i++) //画矩形
    line(back_image, vertices[i], vertices[(i + 1) % 4], Scalar(255, 0, 0), 2);
  // ellipse(back_image, ro, Scalar(0, 0, 255), 2);
}

Mat Draw(std::vector<Point2f> &obs_list, PoseSE2 &robot_current_pose)
{
  Mat background = Mat::ones(rows, cols, CV_8UC3);
  for (int i = 0; i < rows; i++)
    for (int j = 0; j < cols; j++)
      for (int k = 0; k < 3; k++)
        background.at<Vec3b>(i, j)[k] = 255;
  Point2f tp1(Coor_Zoom(0, 0)), tp2(Coor_Zoom(25, 6));
  tp1.x -= 4;
  tp1.y += 4;
  tp2.x += 4;
  tp2.y -= 4;
  rectangle(background, tp1, tp2, Scalar(0, 0, 0), 2);
  Draw_Obs(background, obs_list, robot_current_pose);
  // for (int i = 0; i < robot_position_list.size() - 1; i++)
  //       line(background,robot_position_list[i],robot_position_list[i+1],Scalar(255,0,255));
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

void draw_Poses(PoseSequence &poses, Mat &background)
{
  for (int i = 0; i < poses.size() - 1; i++)
  {
    line(background, Point2f(poses.at(i)->pose().x(), poses.at(i)->pose().y()), Point2f(poses.at(i + 1)->pose().x(), poses.at(i + 1)->pose().y()), Scalar(255, 0, 0));
  }
}

void draw_Poses(TimedElasticBand &teb, Mat &background)
{
  for (unsigned int i = 0; i < teb.sizePoses() - 1; i++)
  {
    Point2f tp1 = Coor_Zoom(teb.Pose(i).x(), teb.Pose(i).y());
    Point2f tp2 = Coor_Zoom(teb.Pose(i + 1).x(), teb.Pose(i + 1).y());
    line(background, tp1, tp2, Scalar(0, 0, 255));
  }
}

Mat Draw_Arc(Mat back, Point2f robot, double yaw, double num)
{
  Point2f trans = Coor_Zoom(robot.x, robot.y);
  Mat dst;
  back.copyTo(dst);
  Point pt[1][42];
  float angle = -60.0;
  pt[0][0] = Point(trans.x, trans.y);
  int i = 1;
  while (angle <= 60.0)
  {
    pt[0][i] = Point(trans.x + num * vt * cos(yaw + angle / 180.0 * PI), trans.y + num * vt * sin(yaw + angle / 180.0 * PI));
    i++;
    angle += 3.0;
  }
  const Point *ppt[1] = {pt[0]};
  int npt[1] = {42};
  fillPoly(back, ppt, npt, 1, Scalar(255, 0, 0));
  addWeighted(dst, 0.7, back, 0.3, 0, dst);
  return dst;
}

Point2f relative_pose(PoseSE2 robot, Point2f obs_position)
{
  float x = (obs_position.x - robot.x()) * cos(robot.theta());
  float y = (obs_position.y - robot.y()) * sin(robot.theta());
  return Point2f(x, y);
}

double cal_dist(Point2f &p1, Point2f &p2)
{
  double re = sqrt(pow(p1.x - p2.x, 2.0) + pow(p1.y - p2.y, 2.0));
  return re;
}

void init_Pose_list(vector<PoseSE2> global_plan, Point2f &start, Point2f &end)
{
  vector<PoseSE2> vtTemp;
  vtTemp.swap(global_plan);
  double dist = cal_dist(start, end);
  double n = 0;
  while (n < dist)
  {
    double dx = end.x - start.x, dy = end.y - start.y;
    double angle = atan2(dy, dx);
    PoseSE2 tp(start.x + n * dx / dist, start.y + n * dy / dist, angle);
    global_plan.push_back(tp);
    n += 0.025;
  }
}

std::vector<geometry_msgs::PoseStamped> global_plan_message(Point2f &start, Point2f &end)
{
  std::vector<geometry_msgs::PoseStamped> global_message;
  double dist = cal_dist(start, end);
  double n = 0;
  while (n < dist)
  {
    double dx = end.x - start.x, dy = end.y - start.y;
    double angle = atan2(dy, dx);
    PoseSE2 tp(start.x + n * dx / dist, start.y + n * dy / dist, angle);
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = config.map_frame;
    pose.header.stamp = ros::Time::now();
    pose.pose.position.x = start.x + n * dx / dist;
    pose.pose.position.y = start.y + n * dy / dist;
    pose.pose.position.z = 0;
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(angle);
    global_message.push_back(pose);
    n += 0.02;
  }
  return global_message;
}
