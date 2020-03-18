

#ifndef OPTIMAL_PLANNER_H_
#define OPTIMAL_PLANNER_H_

#include <math.h>


// teb stuff
#include <teb_local_planner/teb_config.h>
#include <teb_local_planner/misc.h>
#include <teb_local_planner/timed_elastic_band.h>
#include <teb_local_planner/planner_interface.h>
#include <teb_local_planner/visualization.h>
#include <teb_local_planner/robot_footprint_model.h>

// g2o lib stuff
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"

// g2o custom edges and vertices for the TEB planner
#include <teb_local_planner/g2o_types/edge_velocity.h>
#include <teb_local_planner/g2o_types/edge_acceleration.h>
#include <teb_local_planner/g2o_types/edge_kinematics.h>
#include <teb_local_planner/g2o_types/edge_time_optimal.h>
#include <teb_local_planner/g2o_types/edge_shortest_path.h>
#include <teb_local_planner/g2o_types/edge_obstacle.h>
#include <teb_local_planner/g2o_types/edge_dynamic_obstacle.h>
#include <teb_local_planner/g2o_types/edge_via_point.h>
#include <teb_local_planner/g2o_types/edge_prefer_rotdir.h>

// messages
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <teb_local_planner/TrajectoryMsg.h>

#include <nav_msgs/Odometry.h>
#include <limits.h>

namespace teb_local_planner
{

//! Typedef for the block solver utilized for optimization
typedef g2o::BlockSolver< g2o::BlockSolverTraits<-1, -1> >  TEBBlockSolver;

//! Typedef for the linear solver utilized for optimization
typedef g2o::LinearSolverCSparse<TEBBlockSolver::PoseMatrixType> TEBLinearSolver;
//typedef g2o::LinearSolverCholmod<TEBBlockSolver::PoseMatrixType> TEBLinearSolver;

//! Typedef for a container storing via-points
typedef std::vector< Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > ViaPointContainer;


/**
 * @class TebOptimalPlanner
 * @brief This class optimizes an internal Timed Elastic Band trajectory using the g2o-framework.
 * 
 * For an introduction and further details about the TEB optimization problem refer to:
 * 	- C. Rösmann et al.: Trajectory modification considering dynamic constraints of autonomous robots, ROBOTIK, 2012.
 * 	- C. Rösmann et al.: Efficient trajectory optimization using a sparse model, ECMR, 2013.
 * 	- R. Kümmerle et al.: G2o: A general framework for graph optimization, ICRA, 2011. 
 * 
 * @todo: Call buildGraph() only if the teb structure has been modified to speed up hot-starting from previous solutions.
 * @todo: We introduced the non-fast mode with the support of dynamic obstacles
 *        (which leads to better results in terms of x-y-t homotopy planning).
 *        However, we have not tested this mode intensively yet, so we keep
 *        the legacy fast mode as default until we finish our tests.
 */
class TebOptimalPlanner : public PlannerInterface
{
public:
    
  TebOptimalPlanner();
  
  TebOptimalPlanner(const TebConfig& cfg, ObstContainer* obstacles = NULL, RobotFootprintModelPtr robot_model = boost::make_shared<PointRobotFootprint>(),
                    TebVisualizationPtr visual = TebVisualizationPtr(), const ViaPointContainer* via_points = NULL);
  virtual ~TebOptimalPlanner();
  void initialize(const TebConfig& cfg, ObstContainer* obstacles = NULL, RobotFootprintModelPtr robot_model = boost::make_shared<PointRobotFootprint>(),
                  TebVisualizationPtr visual = TebVisualizationPtr(), const ViaPointContainer* via_points = NULL);
  virtual bool plan(const std::vector<geometry_msgs::PoseStamped>& initial_plan, const geometry_msgs::Twist* start_vel = NULL, bool free_goal_vel=false);
  
  virtual bool plan(const tf::Pose& start, const tf::Pose& goal, const geometry_msgs::Twist* start_vel = NULL, bool free_goal_vel=false);
  
  virtual bool plan(const PoseSE2& start, const PoseSE2& goal, const geometry_msgs::Twist* start_vel = NULL, bool free_goal_vel=false);
  
  virtual bool getVelocityCommand(double& vx, double& vy, double& omega) const;
    
  bool optimizeTEB(int iterations_innerloop, int iterations_outerloop, bool compute_cost_afterwards = false,
                   double obst_cost_scale=1.0, double viapoint_cost_scale=1.0, bool alternative_time_cost=false);
  void setVelocityStart(const geometry_msgs::Twist& vel_start);
  
  void setVelocityGoal(const geometry_msgs::Twist& vel_goal);
 
  void setVelocityGoalFree() {vel_goal_.first = false;}
  
  void setObstVector(ObstContainer* obst_vector) {obstacles_ = obst_vector;}
  
  const ObstContainer& getObstVector() const {return *obstacles_;}

  void setViaPoints(const ViaPointContainer* via_points) {via_points_ = via_points;}
  const ViaPointContainer& getViaPoints() const {return *via_points_;}
  void setVisualization(TebVisualizationPtr visualization);

  virtual void visualize();
  
  virtual void clearPlanner() 
  {
    clearGraph();
    teb_.clearTimedElasticBand();
  }
  
  virtual void setPreferredTurningDir(RotType dir) {prefer_rotdir_=dir;}
  static void registerG2OTypes();

  TimedElasticBand& teb() {return teb_;};

PoseSequence poses() {return teb_.poses();};
  /**

   * @brief Access the internal TimedElasticBand trajectory (read-only).
   * @return const reference to the teb
   */
  const TimedElasticBand& teb() const {return teb_;};
  
  /**
   * @brief Access the internal g2o optimizer.
   * @warning In general, the underlying optimizer must not be modified directly. Use with care...
   * @return const shared pointer to the g2o sparse optimizer
   */
  boost::shared_ptr<g2o::SparseOptimizer> optimizer() {return optimizer_;};
  
  /**
   * @brief Access the internal g2o optimizer (read-only).
   * @return const shared pointer to the g2o sparse optimizer
   */
  boost::shared_ptr<const g2o::SparseOptimizer> optimizer() const {return optimizer_;};
  
  /**
   * @brief Check if last optimization was successful
   * @return \c true if the last optimization returned without errors, 
   *         otherwise \c false (also if no optimization has been called before).
   */
  bool isOptimized() const {return optimized_;};
	
  /**
   * @brief Compute the cost vector of a given optimization problen (hyper-graph must exist).
   * 
   * Use this method to obtain information about the current edge errors / costs (local cost functions). \n
   * The vector of cost values is composed according to the different edge types (time_optimal, obstacles, ...). \n
   * Refer to the method declaration for the detailed composition. \n
   * The cost for the edges that minimize time differences (EdgeTimeOptimal) corresponds to the sum of all single
   * squared time differneces: \f$ \sum_i \Delta T_i^2 \f$. Sometimes, the user may want to get a value that is proportional
   * or identical to the actual trajectory transition time \f$ \sum_i \Delta T_i \f$. \n
   * Set \c alternative_time_cost to true in order to get the cost calculated using the latter equation, but check the 
   * implemented definition, if the value is scaled to match the magnitude of other cost values.
   * 
   * @todo Remove the scaling term for the alternative time cost.
   * @todo Can we use the last error (chi2) calculated from g2o instead of calculating it by ourself?
   * @see getCurrentCost
   * @see optimizeTEB
   * @param obst_cost_scale Specify extra scaling for obstacle costs.
   * @param viapoint_cost_scale Specify extra scaling for via points.
   * @param alternative_time_cost Replace the cost for the time optimal objective by the actual (weighted) transition time.
   * @return TebCostVec containing the cost values
   */
  void computeCurrentCost(double obst_cost_scale=1.0, double viapoint_cost_scale=1.0, bool alternative_time_cost=false);
  
  /**
   * Compute and return the cost of the current optimization graph (supports multiple trajectories)
   * @param[out] cost current cost value for each trajectory
   *                  [for a planner with just a single trajectory: size=1, vector will not be cleared]
   * @param obst_cost_scale Specify extra scaling for obstacle costs
   * @param viapoint_cost_scale Specify extra scaling for via points.
   * @param alternative_time_cost Replace the cost for the time optimal objective by the actual (weighted) transition time
   */
  virtual void computeCurrentCost(std::vector<double>& cost, double obst_cost_scale=1.0, double viapoint_cost_scale=1.0, bool alternative_time_cost=false)
  {
    computeCurrentCost(obst_cost_scale, viapoint_cost_scale, alternative_time_cost);
    cost.push_back( getCurrentCost() );
  }
  
  /**
   * @brief Access the cost vector.
   *
   * The accumulated cost value previously calculated using computeCurrentCost 
   * or by calling optimizeTEB with enabled cost flag.
   * @return const reference to the TebCostVec.
   */
  double getCurrentCost() const {return cost_;}
  
    
  /**
   * @brief Extract the velocity from consecutive poses and a time difference (including strafing velocity for holonomic robots)
   * 
   * The velocity is extracted using finite differences.
   * The direction of the translational velocity is also determined.
   * @param pose1 pose at time k
   * @param pose2 consecutive pose at time k+1
   * @param dt actual time difference between k and k+1 (must be >0 !!!)
   * @param[out] vx translational velocity
   * @param[out] vy strafing velocity which can be nonzero for holonomic robots
   * @param[out] omega rotational velocity
   */
  inline void extractVelocity(const PoseSE2& pose1, const PoseSE2& pose2, double dt, double& vx, double& vy, double& omega) const;
  
  /**
   * @brief Compute the velocity profile of the trajectory
   * 
   * This method computes the translational and rotational velocity for the complete
   * planned trajectory. 
   * The first velocity is the one that is provided as initial velocity (fixed).
   * Velocities at index k=2...end-1 are related to the transition from pose_{k-1} to pose_k. 
   * The last velocity is the final velocity (fixed).
   * The number of Twist objects is therefore sizePoses()+1;
   * In summary:
   *     v[0] = v_start,
   *     v[1,...end-1] = +-(pose_{k+1}-pose{k})/dt, 
   *     v(end) = v_goal
   * It can be used for evaluation and debugging purposes or
   * for open-loop control. For computing the velocity required for controlling the robot
   * to the next step refer to getVelocityCommand().
   * @param[out] velocity_profile velocity profile will be written to this vector (after clearing any existing content) with the size=no_poses+1
   */
  void getVelocityProfile(std::vector<geometry_msgs::Twist>& velocity_profile) const;
  
    /**
   * @brief Return the complete trajectory including poses, velocity profiles and temporal information
   * 
   * It is useful for evaluation and debugging purposes or for open-loop control.
   * Since the velocity obtained using difference quotients is the mean velocity between consecutive poses,
   * the velocity at each pose at time stamp k is obtained by taking the average between both velocities.
   * The velocity of the first pose is v_start (provided initial value) and the last one is v_goal (usually zero, if free_goal_vel is off).
   * See getVelocityProfile() for the list of velocities between consecutive points.
   * @todo The acceleration profile is not added at the moment.
   * @param[out] trajectory the resulting trajectory
   */
  void getFullTrajectory(std::vector<TrajectoryPointMsg>& trajectory) const;
  
  /**
   * @brief Check whether the planned trajectory is feasible or not.
   * 
   * This method currently checks only that the trajectory, or a part of the trajectory is collision free.
   * Obstacles are here represented as costmap instead of the internal ObstacleContainer.
   * @param costmap_model Pointer to the costmap model
   * @param footprint_spec The specification of the footprint of the robot in world coordinates
   * @param inscribed_radius The radius of the inscribed circle of the robot
   * @param circumscribed_radius The radius of the circumscribed circle of the robot
   * @param look_ahead_idx Number of poses along the trajectory that should be verified, if -1, the complete trajectory will be checked.
   * @return \c true, if the robot footprint along the first part of the trajectory intersects with 
   *         any obstacle in the costmap, \c false otherwise.
   */
  virtual bool isTrajectoryFeasible(base_local_planner::CostmapModel* costmap_model, const std::vector<geometry_msgs::Point>& footprint_spec, double inscribed_radius = 0.0,
          double circumscribed_radius=0.0, int look_ahead_idx=-1);
  
  
  /**
   * @brief Check if the planner suggests a shorter horizon (e.g. to resolve problems)
   * 
   * This method is intendend to be called after determining that a trajectory provided by the planner is infeasible.
   * In some cases a reduction of the horizon length might resolve problems. E.g. if a planned trajectory cut corners.
   * Implemented cases for returning \c true (remaining length must be larger than 2m to trigger any case):
   * - Goal orientation - start orientation > 90°
   * - Goal heading - start orientation > 90°
   * - The planned trajectory is at least 30° shorter than the initial plan (accumulated euclidean distances)
   * - Distance between consecutive poses > 0.9*min_obstacle_dist
   * @param initial_plan The intial and transformed plan (part of the local map and pruned up to the robot position)
   * @return \c true, if the planner suggests a shorter horizon, \c false otherwise.
   */
  virtual bool isHorizonReductionAppropriate(const std::vector<geometry_msgs::PoseStamped>& initial_plan) const;
  
  //@}
  
protected:
  
  /** @name Hyper-Graph creation and optimization */
  //@{
  
  /**
   * @brief Build the hyper-graph representing the TEB optimization problem.
   * 
   * This method creates the optimization problem according to the hyper-graph formulation. \n
   * For more details refer to the literature cited in the TebOptimalPlanner class description.
   * @see optimizeGraph
   * @see clearGraph
   * @param weight_multiplier Specify a weight multipler for selected weights in optimizeGraph
   *                          This might be used for weight adapation strategies.
   *                          Currently, only obstacle collision weights are considered.
   * @return \c true, if the graph was created successfully, \c false otherwise.
   */
  bool buildGraph(double weight_multiplier=1.0);
  
  /**
   * @brief Optimize the previously constructed hyper-graph to deform / optimize the TEB.
   * 
   * This method invokes the g2o framework to solve the optimization problem considering dedicated sparsity patterns. \n
   * The current implementation calls a non-constrained sparse Levenberg-Marquardt algorithm. Constraints are considered
   * by utilizing penalty approximations. Refer to the literature cited in the TebOptimalPlanner class description.
   * @see buildGraph
   * @see clearGraph
   * @param no_iterations Number of solver iterations
   * @param clear_after Clear the graph after optimization.
   * @return \c true, if optimization terminates successfully, \c false otherwise.
   */
  bool optimizeGraph(int no_iterations, bool clear_after=true);
  
  /**
   * @brief Clear an existing internal hyper-graph.
   * @see buildGraph
   * @see optimizeGraph
   */
  void clearGraph();
  
  /**
   * @brief Add all relevant vertices to the hyper-graph as optimizable variables.
   * 
   * Vertices (if unfixed) represent the variables that will be optimized. \n
   * In case of the Timed-Elastic-Band poses and time differences form the vertices of the hyper-graph. \n
   * The order of insertion of vertices (to the graph) is important for efficiency,
   * since it affect the sparsity pattern of the underlying hessian computed for optimization.
   * @see VertexPose
   * @see VertexTimeDiff
   * @see buildGraph
   * @see optimizeGraph
   */
  void AddTEBVertices();
  
  /**
   * @brief Add all edges (local cost functions) for limiting the translational and angular velocity.
   * @see EdgeVelocity
   * @see buildGraph
   * @see optimizeGraph
   */
  void AddEdgesVelocity();
  
  /**
   * @brief Add all edges (local cost functions) for limiting the translational and angular acceleration.
   * @see EdgeAcceleration
   * @see EdgeAccelerationStart
   * @see EdgeAccelerationGoal
   * @see buildGraph
   * @see optimizeGraph
   */
  void AddEdgesAcceleration();
  
  /**
   * @brief Add all edges (local cost functions) for minimizing the transition time (resp. minimize time differences)
   * @see EdgeTimeOptimal
   * @see buildGraph
   * @see optimizeGraph
   */
  void AddEdgesTimeOptimal();

  /**
   * @brief Add all edges (local cost functions) for minimizing the path length
   * @see EdgeShortestPath
   * @see buildGraph
   * @see optimizeGraph
   */
  void AddEdgesShortestPath();
  
  /**
   * @brief Add all edges (local cost functions) related to keeping a distance from static obstacles
   * @warning do not combine with AddEdgesInflatedObstacles
   * @see EdgeObstacle
   * @see buildGraph
   * @see optimizeGraph
   * @param weight_multiplier Specify an additional weight multipler (in addition to the the config weight)
   */
  void AddEdgesObstacles(double weight_multiplier=1.0);
  
  /**
   * @brief Add all edges (local cost functions) related to keeping a distance from static obstacles (legacy association strategy)
   * @warning do not combine with AddEdgesInflatedObstacles
   * @see EdgeObstacle
   * @see buildGraph
   * @see optimizeGraph
   * @param weight_multiplier Specify an additional weight multipler (in addition to the the config weight)
   */
  void AddEdgesObstaclesLegacy(double weight_multiplier=1.0);
  
  /**
   * @brief Add all edges (local cost functions) related to minimizing the distance to via-points
   * @see EdgeViaPoint
   * @see buildGraph
   * @see optimizeGraph
   */
  void AddEdgesViaPoints();
  
  /**
   * @brief Add all edges (local cost functions) related to keeping a distance from dynamic (moving) obstacles.
   * @warning experimental 
   * @todo Should we also add neighbors to decrease jiggling/oscillations
   * @see EdgeDynamicObstacle
   * @see buildGraph
   * @see optimizeGraph
   * @param weight_multiplier Specify an additional weight multipler (in addition to the the config weight)

   */
  void AddEdgesDynamicObstacles(double weight_multiplier=1.0);

  /**
   * @brief Add all edges (local cost functions) for satisfying kinematic constraints of a differential drive robot
   * @warning do not combine with AddEdgesKinematicsCarlike()
   * @see AddEdgesKinematicsCarlike
   * @see buildGraph
   * @see optimizeGraph
   */
  void AddEdgesKinematicsDiffDrive();
  
  /**
   * @brief Add all edges (local cost functions) for satisfying kinematic constraints of a carlike robot
   * @warning do not combine with AddEdgesKinematicsDiffDrive()
   * @see AddEdgesKinematicsDiffDrive
   * @see buildGraph
   * @see optimizeGraph
   */
  void AddEdgesKinematicsCarlike();
  
  /**
   * @brief Add all edges (local cost functions) for prefering a specifiy turning direction (by penalizing the other one)
   * @see buildGraph
   * @see optimizeGraph
   */
  void AddEdgesPreferRotDir(); 
  
  //@}
  
  
  /**
   * @brief Initialize and configure the g2o sparse optimizer.
   * @return shared pointer to the g2o::SparseOptimizer instance
   */
  boost::shared_ptr<g2o::SparseOptimizer> initOptimizer();
    

  // external objects (store weak pointers)
  const TebConfig* cfg_; //!< Config class that stores and manages all related parameters
  ObstContainer* obstacles_; //!< Store obstacles that are relevant for planning
  const ViaPointContainer* via_points_; //!< Store via points for planning
  
  double cost_; //!< Store cost value of the current hyper-graph
  RotType prefer_rotdir_; //!< Store whether to prefer a specific initial rotation in optimization (might be activated in case the robot oscillates)
  
  // internal objects (memory management owned)
  TebVisualizationPtr visualization_; //!< Instance of the visualization class
  TimedElasticBand teb_; //!< Actual trajectory object
  RobotFootprintModelPtr robot_model_; //!< Robot model
  boost::shared_ptr<g2o::SparseOptimizer> optimizer_; //!< g2o optimizer for trajectory optimization
  std::pair<bool, geometry_msgs::Twist> vel_start_; //!< Store the initial velocity at the start pose
  std::pair<bool, geometry_msgs::Twist> vel_goal_; //!< Store the final velocity at the goal pose

  bool initialized_; //!< Keeps track about the correct initialization of this class
  bool optimized_; //!< This variable is \c true as long as the last optimization has been completed successful
  
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW    
};

//! Abbrev. for shared instances of the TebOptimalPlanner
typedef boost::shared_ptr<TebOptimalPlanner> TebOptimalPlannerPtr;
//! Abbrev. for shared const TebOptimalPlanner pointers
typedef boost::shared_ptr<const TebOptimalPlanner> TebOptimalPlannerConstPtr;
//! Abbrev. for containers storing multiple teb optimal planners
typedef std::vector< TebOptimalPlannerPtr > TebOptPlannerContainer;

} // namespace teb_local_planner

#endif /* OPTIMAL_PLANNER_H_ */
