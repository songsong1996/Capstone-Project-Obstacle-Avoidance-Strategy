
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
#include<math.h>

using namespace teb_local_planner; // it is ok here to import everything for testing purposes

using namespace cv;
using namespace std;
//============== Class Definations ===============

// ============= Global Variables ================
// Ok global variables are bad, but here we only have a simple testing node.
HomotopyClassPlanner planner;
TebVisualizationPtr visual;
std::vector<ObstaclePtr> obst_vector;
ViaPointContainer via_points;
TebConfig config;
boost::shared_ptr<dynamic_reconfigure::Server<TebLocalPlannerReconfigureConfig>> dynamic_recfg;
ros::Subscriber custom_obst_sub;
ros::Subscriber via_points_sub;
ros::Subscriber clicked_points_sub;

std::vector<ros::Subscriber> obs_vel_subs;
unsigned int no_fixed_obstacles;
ros::Subscriber robot_odom, simple_goal;
std::vector<Point2f> obs_list;
std::vector<Point2f> robot_position_list;
std::vector<PoseSE2> robot_pose_list;
Point2f start_point, goal_point, robot_position;
PoseSE2 robot_current_pose;
double scale = 30;
int rec_width = 25;
int rec_height = 6;
int all_width = 28;
int all_height = 8;
int rows = all_height * scale;
int cols = all_width * scale;

// =========== Function declarations =============
void CB_mainCycle(const ros::TimerEvent &e);
void CB_publishCycle(const ros::TimerEvent &e);
void CB_reconfigure(TebLocalPlannerReconfigureConfig &reconfig, uint32_t level);
void CB_customObstacle(const costmap_converter::ObstacleArrayMsg::ConstPtr &obst_msg);
void CreateInteractiveMarker(const double &init_x, const double &init_y, unsigned int id, std::string frame, interactive_markers::InteractiveMarkerServer *marker_server, interactive_markers::InteractiveMarkerServer::FeedbackCallback feedback_cb);
void CB_obstacle_marker(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
void CB_clicked_points(const geometry_msgs::PointStampedConstPtr &point_msg);
void CB_via_points(const nav_msgs::Path::ConstPtr &via_points_msg);
void CB_setObstacleVelocity(const geometry_msgs::TwistConstPtr &twist_msg, const unsigned int id);

void CB_obstacle_point(const nav_msgs::Odometry::ConstPtr &msg, const unsigned int id);
void CB_RobotPosition(const nav_msgs::Odometry::ConstPtr &msg);
void CB_getGoals(const geometry_msgs::PoseStamped::ConstPtr &msg);

//void publishLocalPlanAndPoses(TimedElasticBand &teb, TebConfig &config);

//draw
Point2f Coor_Zoom(double x, double y);
void Draw_Obs(Mat &back_image, std::vector<Point2f> &obs_list, Point2f &robot);
Mat Draw(std::vector<Point2f> &obs_list, Point2f &robot);
Mat init_Back_Image();
void draw_Poses(PoseSequence &poses, Mat &background);
void draw_Poses(TimedElasticBand &teb, Mat &background);
int count_num=0;

// =============== Main function =================
int main(int argc, char **argv)
{
  ros::init(argc, argv, "subscriber2");
  ros::NodeHandle n;
  // load ros parameters from node handle
  config.loadRosParamFromNodeHandle(n);

  ros::Timer cycle_timer = n.createTimer(ros::Duration(0.025), CB_mainCycle);

  for (int i = 0; i < 13; i++)
  {
    obs_list.push_back(Point2f(0, 0));
    obst_vector.push_back(boost::make_shared<PointObstacle>(0, 0));
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
  RobotFootprintModelPtr robot_model = TebLocalPlannerROS::getRobotFootprintFromParamServer(n);
  // Setup planner (homotopy class planning or just the local teb planner)
  if (config.hcp.enable_homotopy_class_planning)
    // planner = PlannerInterfacePtr(new HomotopyClassPlanner(config, &obst_vector, robot_model));
    planner = HomotopyClassPlanner(config, &obst_vector, robot_model, visual);

  ros::spin();
  //cout << "===========================" << endl;
  return 0;
}

// Planning loop
void CB_mainCycle(const ros::TimerEvent &e)
{
  // planner.plan(PoseSE2(0,0, 0), PoseSE2(5,0, 0));
  planner.plan(PoseSE2(robot_position.x, robot_position.y, 0), PoseSE2(goal_point.x, goal_point.y, 0)); // hardcoded start and goal for testing purposes
  Mat back1 = Draw(obs_list, robot_position);

  cout << goal_point.x << "   " << robot_position.x << "  " << goal_point.y << "  " << robot_position.y << endl;
  TebOptimalPlannerPtr teb_optim_planner_ptr = planner.bestTeb();
  // cout<<"teb_ptr"<<endl;

  if (teb_optim_planner_ptr)
  {
    TimedElasticBand &teb_band = teb_optim_planner_ptr->teb();
   // cout << "teb_band" << endl;
    draw_Poses(teb_band, back1);
  }
  
  // cout<<"poses"<<endl;
  imshow("tp", back1);
  waitKey(10);
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

void CB_clicked_points(const geometry_msgs::PointStampedConstPtr &point_msg)
{
  // we assume for simplicity that the fixed frame is already the map/planning frame
  // consider clicked points as via-points
  via_points.push_back(Eigen::Vector2d(point_msg->point.x, point_msg->point.y));
  ROS_INFO_STREAM("Via-point (" << point_msg->point.x << "," << point_msg->point.y << ") added.");
  if (config.optim.weight_viapoint <= 0)
    ROS_WARN("Note, via-points are deactivated, since 'weight_via_point' <= 0");
}

void CB_via_points(const nav_msgs::Path::ConstPtr &via_points_msg)
{
  ROS_INFO_ONCE("Via-points received. This message is printed once.");
  via_points.clear();
  for (const geometry_msgs::PoseStamped &pose : via_points_msg->poses)
  {
    via_points.emplace_back(pose.pose.position.x, pose.pose.position.y);
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
  obst_vector[id - 1] = boost::make_shared<PointObstacle>(msg->pose.pose.position.x, msg->pose.pose.position.y);
  //cout<<id<<endl;
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
  //cout << robot_position.x << "  " << robot_position.y << endl;
  robot_position_list.push_back(robot_position);
  
  PoseSE2 tp_pose(msg->pose.pose.position.x,msg->pose.pose.position.y,yaw);
  robot_current_pose=PoseSE2(msg->pose.pose.position.x,msg->pose.pose.position.y,yaw);
  robot_pose_list.push_back(tp_pose);
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
  return Point2f(xx, yy);
}

void Draw_Obs(Mat &back_image, std::vector<Point2f> &obs_list, Point2f &robot)
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
  Point2f trans = Coor_Zoom(robot.x, robot.y);
  double ro_size = 4;
  //Point2f tp1(trans.x - ro_size, trans.y - ro_size), tp2(trans.x + ro_size, trans.y + ro_size);
  //rectangle(back_image, tp1, tp2, Scalar(255, 0, 0), 2);
  RotatedRect ro(trans, Size2f(ro_size * 2, ro_size * 2), robot_current_pose.theta());
  Point2f vertices[4];
    ro.points(vertices);
    for (int i = 0; i < 4; i++)//画矩形
        line(back_image , vertices[i], vertices[(i + 1) % 4], Scalar(0, 255, 255));
  // ellipse(back_image, ro, Scalar(0, 0, 255), 2);
}

Mat Draw(std::vector<Point2f> &obs_list, Point2f &robot)
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
  Draw_Obs(background, obs_list, robot);
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
    line(background, Point2f(teb.Pose(i).x(), teb.Pose(i).y()), Point2f(teb.Pose(i + 1).x(), teb.Pose(i + 1).y()), Scalar(255, 0, 0));
  }
}
// void publishLocalPlanAndPoses(TimedElasticBand &teb, TebConfig config)
// {
//   // create path msg
//   nav_msgs::Path teb_path;
//   teb_path.header.frame_id = config->map_frame;
//   teb_path.header.stamp = ros::Time::now();

//   // create pose_array (along trajectory)
//   geometry_msgs::PoseArray teb_poses;
//   teb_poses.header.frame_id = teb_path.header.frame_id;
//   teb_poses.header.stamp = teb_path.header.stamp;

//   // fill path msgs with teb configurations
//   for (unsigned int i = 0; i < teb.sizePoses(); i++)
//   {
//     geometry_msgs::PoseStamped pose;
//     pose.header.frame_id = teb_path.header.frame_id;
//     pose.header.stamp = teb_path.header.stamp;
//     pose.pose.position.x = teb.Pose(i).x();
//     pose.pose.position.y = teb.Pose(i).y();
//     pose.pose.position.z = 0;
//     pose.pose.orientation = tf::createQuaternionMsgFromYaw(teb.Pose(i).theta());
//     teb_path.poses.push_back(pose);
//     teb_poses.poses.push_back(pose.pose);
//   }
//   local_plan_pub_.publish(teb_path);
//   teb_poses_pub_.publish(teb_poses);
// }


Point2f relative_pose(PoseSE2 robot,Point2f obs_position)
{
  float x=(obs_position.x-robot.x())*cos(robot.theta()) ;
  float y=(obs_position.y-robot.y())*sin(robot.theta()) ;
  return Point2f(x,y);
}

