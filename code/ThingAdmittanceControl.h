/*
 * ThingAdmittanceControl.h
 *
 *  Created on: Mar 9, 2018
 *      Author: ulrichto
 */

#ifndef INCLUDE_THING_ADMITTANCE_CONTROL_THINGADMITTANCECONTROL_H_
#define INCLUDE_THING_ADMITTANCE_CONTROL_THINGADMITTANCECONTROL_H_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <math.h>
#include <array>
#include <numeric>      // std::accumulate
#include <iterator>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_datatypes.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Float64MultiArray.h>
#include <iostream>

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

//#include <tf/exception.h>

typedef Eigen::Matrix<double, 7, 1> Vector7d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;

using actionlib::SimpleActionClient;

using namespace Eigen;

namespace thing_admittance_control {

class ThingAdmittanceControl {
 public:
  /*!
   * Constructor.
   */
  ThingAdmittanceControl(ros::NodeHandle &nodeHandle,
                         std::string wrench_input_topic,
                         std::string wrench_control_topic,
                         std::string platform_state_topic,
                         std::string pose_command_topic,
                         std::string pose_platform_topic,
                         std::string pose_arm_topic, std::string pose_eq_topic,
                         std::string eq_error_topic,
                         std::string twist_platform_topic,
                         std::string twist_arm_topic, double loop_rate,
                         bool pose_output, std::string ee_frame,
                         std::string ft_frame, std::string armbase_frame,
                         std::string base_frame, std::string world_frame,
                         std::vector<double> M_p, std::vector<double> M_a,
                         std::vector<double> D, std::vector<double> D_p,
                         std::vector<double> D_a, std::vector<double> K,
                         std::vector<double> eq_pose,
                         double wrench_filter_factor,
                         double force_dead_zone_thres,
                         double torque_dead_zone_thres, double P_base_pos,
                         double P_arm_pos, double P_wrench_obs,
                         int D_wrench_obs);
  /*!
   * Destructor.
   */
  virtual ~ThingAdmittanceControl();

  // Executor
  void execute();

 private:
  //***************
  //VARIABLES
  //***************

  // Frames
  std::string ee_frame_, ft_frame_, armbase_frame_, base_frame_, world_frame_;

  ///////////////////
  // ROS VARIABLES //
  ///////////////////
  ros::NodeHandle nodeHandle_;
  ros::Rate loop_rate_;  // Rate of the execution loop
  ros::Time traj_loop_duration_;

  // Subscribers
  ros::Subscriber wrench_sub_, wrench_control_sub_;
  ros::Subscriber base_sub_;
  ros::Subscriber pose_des_base_obs_avoid_sub;
  ros::Subscriber base_planner_sub_;
  ros::Subscriber local_costmap_sub_;

  // Publishers
  ros::Publisher wrench_e_pub_, wrench_obs_pub_;
  ros::Publisher pose_state_arm_pub_;
  ros::Publisher pose_des_base_pub_;  //Eigen Publisher for obstacle avoidance
  ros::Publisher pose_cmd_pub_;  //Publisher for Thing Control
  ros::Publisher pose_cmd_base_pub_, pose_cmd_base_dir_pub_;
  ros::Publisher pose_cmd_arm_pub_, pose_cmd_arm_dir_pub_;
  ros::Publisher point_eq_error_pub_;  //Distance from current point to equilibrium point
  ros::Publisher pose_eq_pub_;  //Equilibrium point of spring mass damper system
  ros::Publisher eq_displacement_pub_;
  ros::Publisher traj_base_pub_, traj_arm_pub_;
  ros::Publisher twist_cmd_base_pub_, twist_cmd_arm_pub_;

  /////////////////////
  // EIGEN VARIABLES //
  /////////////////////
  //                             (in platform base_link)
  // x_a_position_, x_a_orientation_, x_dot_a_, x_ddot_a ->
  //                             Arm state and time derivatives
  //                             (in ur10_arm_base_link frame)
  Vector3d x_a_position_, x_p_position_;
  Quaterniond x_a_orientation_, x_p_orientation_;
  Vector6d x_a_;
  Vector6d x_dot_a_, x_dot_p_;  // Arm and platform velocity
  Vector6d wrench_e_, wrench_obs_;  //External wrench and control wrench in arm base frame

  Matrix6d rotation_base_;  // Transform from base_link to
                            // ur5_arm_base_link
  Matrix6d kin_constraints_;  // Derivative of kinematic constraints between the arm and the platform

  // ADMITTANCE PARAMETERS:
  // M_p_, M_a_ -> Desired mass of platform/arm
  // D_ -> Desired damping of the coupling
  // K_ -> Desired Stiffness of the coupling
  // D_p_, D_a_ -> Desired damping of platform/arm
  Matrix6d M_p_, M_a_, D_, D_p_, D_a_, K_;

  Vector3d eq_pos_;  // Equilibrium position of the coupling spring [m]
  Quaterniond eq_orientation_;  // Equilibrium orientation of the coupling spring
  Vector6d eq_error_, eq_rot_vec_;  // Delta between arm pose and equilibrium pose

  Vector6d des_pose_arm_, des_pose_base_;
  Vector6d des_twist_arm_, des_twist_base_;

  ///////////////////////////
  // CONTROLLER PARAMETERS //
  ///////////////////////////
  Vector3d X_base_pose_, Y_base_pose_;  // queue for des base pose filter
  std::list<double> q_base_pose_x_, q_base_pose_y_, q_base_pose_theta_,
      q_wrench_obs_x_, q_wrench_obs_y_;
  double P_base_pos_, P_arm_pos_, P_wrench_obs_;
  int D_wrench_obs_;

  ////////////////////////
  // GEOMETRY VARIABLES //
  ////////////////////////
  geometry_msgs::WrenchStamped wrench_e_msg_, wrench_obs_msg_;  //Testing purposes
  geometry_msgs::PoseStamped pose_cmd_arm_, pose_cmd_arm_world_,
      pose_cmd_arm_ee_;
  geometry_msgs::PoseStamped pose_cmd_base_, pose_cmd_base_world_;
  geometry_msgs::PoseStamped pose_eq_;
  geometry_msgs::PointStamped distance_eq_error_, goal_point_;
  geometry_msgs::TwistStamped twist_cmd_arm_, twist_cmd_base_;
  geometry_msgs::PoseStamped pose_state_base_;
  geometry_msgs::PoseStamped pose_state_arm_, pose_state_arm_world_;
  geometry_msgs::Twist twist_state_arm_;

  visualization_msgs::Marker eq_displacement_, traj_base_, traj_arm_;

  //////////////////
  // TF VARIABLES //
  //////////////////
  // Listeners
  tf::TransformListener listener_;
  tf::TransformListener listener_ft_;
  tf::TransformListener listener_control_;
  tf::TransformListener listener_arm_;
  tf::TransformListener listener_base_;

  // Guards
  bool ft_armbase_ready_;
  //bool armbase_world_ready_;
  bool base_world_ready_;
  bool world_armbase_ready_;
  bool pose_output_;

  // FT FILTER:
  // Parameters for the noisy wrench
  double wrench_filter_factor_;
  double force_dead_zone_thres_;
  double torque_dead_zone_thres_;

  //***************
  //FUNCTIONS
  //***************

  // Initialization
  void initTF();
  void initMarker();

  // Core Functions
  void computeAdmittance(Vector6d &des_pose_base, Vector6d &des_twist_base,
                         Vector6d &des_pose_arm, Vector6d &des_twist_arm,
                         ros::Duration cycle_time);
  void publish_poseCmd(geometry_msgs::PoseStamped &pose_base,
                       geometry_msgs::PoseStamped &pose_arm,
                       Vector6d &pose_des_base);
  void compute_eqPose(Vector3d &eq_pose_current, Vector3d &eq_pose_origin,
                      geometry_msgs::PoseStamped &pose_admittance,
                      geometry_msgs::PoseStamped &pose_planner,
                      std::string &frame);
  void lin_lowpassFilter(double &x, std::list<double> &queue, int &length,
                         double &gain);
  double quadr_lowpassFilter(double &x, double &f_s, double &f_d, Vector3d &X,
                             Vector3d &Y);

  // Visualization
  void visualize_poseCmd(geometry_msgs::PoseStamped &pose_direction,
                         ros::Publisher &pub_dir, ros::Publisher &pub_cmd);
  void visualize_twistCmd(Vector6d &twist, std::string &frame,
                          ros::Publisher &pub);

  // Callback Functions
  void state_baseCallback(const nav_msgs::OdometryConstPtr msg);
  void wrench_inputCallback(const geometry_msgs::WrenchStampedConstPtr msg);
  void local_costmapCallback(const nav_msgs::OccupancyGridConstPtr msg);

  // Utility Functions
  bool getRotation(Matrix6d & rotation_matrix, tf::TransformListener & listener,
                   std::string target_frame, std::string source_frame);
  void getFK(Vector3d &position, Quaterniond &rotation, Vector6d &pose,
             Vector6d &velocity, tf::TransformListener & listener,
             std::string target_frame, std::string source_frame);
  void getTwist(geometry_msgs::Twist &ee_twist_world,
                tf::TransformListener & listener, std::string tracking_frame,
                std::string obs_frame);
  void compute_eqError(Vector3d &x_a_position, Quaterniond &x_a_orientation,
                       Vector3d &eq_position, Quaterniond &eq_orientation,
                       Vector6d &eq_error);
  void convert_to_poseMsg(geometry_msgs::PoseStamped &pose_msg, Vector6d &pose,
                          std::string frame);
  void transformPose(geometry_msgs::PoseStamped &pose_cmd, Vector6d &pose,
                     tf::TransformListener & listener, std::string target_frame,
                     std::string source_frame);
};

} /* namespace */

#endif /* INCLUDE_THING_ADMITTANCE_CONTROL_THINGADMITTANCECONTROL_H_ */
