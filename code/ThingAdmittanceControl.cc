/*
 * ThingAdmittanceControl.cc
 *
 *  Created on: Mar 9, 2018
 *      Author: ulrichto
 */

#include "thing_admittance_control/ThingAdmittanceControl.h"

namespace thing_admittance_control {

// Declaration of Constructor
ThingAdmittanceControl::ThingAdmittanceControl(ros::NodeHandle& nodeHandle,
                                               std::string wrench_e_msg_topic,
                                               std::string wrench_control_topic,
                                               std::string base_state_topic,
                                               std::string pose_command_topic,
                                               std::string pose_base_topic,
                                               std::string pose_arm_topic,
                                               std::string pose_eq_topic,
                                               std::string eq_error_topic,
                                               std::string twist_base_topic,
                                               std::string twist_arm_topic,
                                               double loop_rate,
                                               bool pose_output,
                                               std::string ee_frame,
                                               std::string ft_frame,
                                               std::string armbase_frame,
                                               std::string base_frame,
                                               std::string world_frame,
                                               std::vector<double> M_p,
                                               std::vector<double> M_a,
                                               std::vector<double> D,
                                               std::vector<double> D_p,
                                               std::vector<double> D_a,
                                               std::vector<double> K,
                                               std::vector<double> eq_pose,
                                               double wrench_filter_factor,
                                               double force_dead_zone_thres,
                                               double torque_dead_zone_thres,
                                               double P_base_pos,
                                               double P_arm_pos,
                                               double P_wrench_obs,
                                               int D_wrench_obs)
    : nodeHandle_(nodeHandle),  //set private variables to public variables
      loop_rate_(loop_rate),
      pose_output_(pose_output),
      ee_frame_(ee_frame),
      ft_frame_(ft_frame),
      armbase_frame_(armbase_frame),
      base_frame_(base_frame),
      world_frame_(world_frame),
      M_p_(M_p.data()),
      M_a_(M_a.data()),
      D_(D.data()),
      D_p_(D_p.data()),
      D_a_(D_a.data()),
      K_(K.data()),
      wrench_filter_factor_(wrench_filter_factor),
      force_dead_zone_thres_(force_dead_zone_thres),
      torque_dead_zone_thres_(torque_dead_zone_thres),
      P_base_pos_(P_base_pos),
      P_arm_pos_(P_arm_pos),
      P_wrench_obs_(P_wrench_obs),
      D_wrench_obs_(D_wrench_obs) {

  // Subscribers
  wrench_sub_ = nodeHandle_.subscribe(
      wrench_e_msg_topic, 5, &ThingAdmittanceControl::wrench_inputCallback,
      this, ros::TransportHints().reliable().tcpNoDelay());
  base_sub_ = nodeHandle_.subscribe(
      base_state_topic, 5, &ThingAdmittanceControl::state_baseCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());
  local_costmap_sub_ = nodeHandle_.subscribe(
      "/move_base/local_costmap/costmap", 5,
      &ThingAdmittanceControl::local_costmapCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());

  // Publishers
  wrench_e_pub_ = nodeHandle_.advertise<geometry_msgs::WrenchStamped>(
      "wrench_e_filtered", 10);
  wrench_obs_pub_ = nodeHandle_.advertise<geometry_msgs::WrenchStamped>(
      "wrench_obs", 10);
  pose_cmd_pub_ = nodeHandle_.advertise<std_msgs::Float64MultiArray>(
      pose_command_topic, 1, true);
  pose_des_base_pub_ = nodeHandle_.advertise<std_msgs::Float64MultiArray>(
      "pose_des_base", 1, true);
  pose_cmd_base_pub_ = nodeHandle_.advertise<geometry_msgs::PoseStamped>(
      pose_base_topic, 10);

  pose_cmd_arm_pub_ = nodeHandle_.advertise<geometry_msgs::PoseStamped>(
      pose_arm_topic, 10);
  pose_eq_pub_ = nodeHandle_.advertise<geometry_msgs::PoseStamped>(
      pose_eq_topic, 1);
  pose_cmd_base_dir_pub_ = nodeHandle_.advertise<geometry_msgs::PolygonStamped>(
      "pose_cmd_rb_direction", 1);
  pose_cmd_arm_dir_pub_ = nodeHandle_.advertise<geometry_msgs::PolygonStamped>(
      "pose_cmd_ur10_direction", 1);
  point_eq_error_pub_ = nodeHandle_.advertise<geometry_msgs::PointStamped>(
      eq_error_topic, 10);
  eq_displacement_pub_ = nodeHandle_.advertise<visualization_msgs::Marker>(
      "eq_displacement", 1);
  traj_base_pub_ = nodeHandle.advertise<visualization_msgs::Marker>("traj_base",
                                                                    1);
  traj_arm_pub_ = nodeHandle.advertise<visualization_msgs::Marker>("traj_arm",
                                                                   1);
  twist_cmd_base_pub_ = nodeHandle_.advertise<geometry_msgs::TwistStamped>(
      twist_base_topic, 1);
  twist_cmd_arm_pub_ = nodeHandle_.advertise<geometry_msgs::TwistStamped>(
      twist_arm_topic, 1);

  ft_armbase_ready_ = false;
  base_world_ready_ = false;
  world_armbase_ready_ = false;

  initTF();
  initMarker();

  wrench_e_.setZero();
  wrench_obs_.setZero();

  // Equilibrium pose in armbase frame
  Vector7d eq_pose_(eq_pose.data());
  eq_pos_ << eq_pose_.head(3);
  // Make sure the orientation goal is normalized
  eq_orientation_.coeffs() << eq_pose_.tail(4) / eq_pose_.tail(4).norm();
  // Additionally store in geometry message for publishing
  pose_eq_.header.frame_id = armbase_frame_;
  pose_eq_.pose.position.x = eq_pos_(0);
  pose_eq_.pose.position.y = eq_pos_(1);
  pose_eq_.pose.position.z = eq_pos_(2);
  pose_eq_.pose.orientation.x = eq_orientation_.x();
  pose_eq_.pose.orientation.y = eq_orientation_.y();
  pose_eq_.pose.orientation.z = eq_orientation_.z();
  pose_eq_.pose.orientation.w = eq_orientation_.w();

  // Kinematic constraints between base and arm at the equilibrium
  kin_constraints_.setZero();
  kin_constraints_.topLeftCorner(3, 3).setIdentity();
  kin_constraints_.bottomRightCorner(3, 3).setIdentity();
  // Screw on the z torque axis
  kin_constraints_.topRightCorner(3, 3) << 0, 0, eq_pos_(1), 0, 0, -eq_pos_(0), 0, 0, 0;

}

//Declaration of Destructor
ThingAdmittanceControl::~ThingAdmittanceControl() {
}

/////////////////
/// FUNCTIONS ///
/////////////////

// Execution
void ThingAdmittanceControl::execute() {

  geometry_msgs::Twist ee_twist_world;
  geometry_msgs::Pose ee_pose_world;
  geometry_msgs::PointStamped obs_pub_msg;

// Initialize integrators
  des_pose_arm_.setZero();
  des_twist_arm_.setZero();
  des_pose_base_.setZero();
  des_twist_base_.setZero();

  traj_loop_duration_ = ros::Time::now();

  pose_state_base_.header.frame_id = world_frame_;
  pose_state_arm_.header.frame_id = armbase_frame_;
  twist_cmd_arm_.header.frame_id = armbase_frame_;
  twist_cmd_base_.header.frame_id = base_frame_;
  distance_eq_error_.header.frame_id = ee_frame_;

  while (nodeHandle_.ok()) {

    // DYNAMICS COMPUTATION
    // Output: Current pose (6DOF) and twist (6DOF) of arm EE in armbase frame
    getFK(x_a_position_, x_a_orientation_, x_a_, x_dot_a_, listener_arm_,
          armbase_frame_, ee_frame_);
    convert_to_poseMsg(pose_state_arm_, x_a_, armbase_frame_);

    // Output: Desired pose and twist of arm and base in EigenVectors
    computeAdmittance(des_pose_base_, des_twist_base_, des_pose_arm_,
                      des_twist_arm_, loop_rate_.expectedCycleTime());

    // LOWPASS FILTERING
//    double f_s = 125.0;
//    double f_d = 5.0;
//    des_pose_base_[0] = quadr_lowpassFilter(des_pose_base_[0], f_s, f_d,
//                                            X_base_pose_, Y_base_pose_);

    // Add gain
    des_pose_base_ = des_pose_base_ * P_base_pos_;
    des_pose_arm_ = des_pose_arm_ * P_arm_pos_;

    des_pose_base_ = des_pose_base_.unaryExpr(
        [](double v) {return std::isfinite(v)? v : 0.0;});

    convert_to_poseMsg(pose_cmd_base_, des_pose_base_, base_frame_);
    convert_to_poseMsg(pose_cmd_arm_, des_pose_arm_, armbase_frame_);

    listener_base_.transformPose(world_frame_, pose_cmd_base_,
                                 pose_cmd_base_world_);
    listener_arm_.transformPose(world_frame_, pose_cmd_arm_,
                                pose_cmd_arm_world_);
    listener_arm_.transformPose(ee_frame_, pose_cmd_arm_, pose_cmd_arm_ee_);

    ///////////
    //PUBLISH//
    ///////////

    // Publish pose cmd for thing control
    publish_poseCmd(pose_cmd_base_world_, pose_cmd_arm_world_, des_pose_base_);

    ///////////////
    // VISUALIZE //
    ///////////////

    // Update trajectories at 4Hz
    if (ros::Time::now().toSec() - traj_loop_duration_.toSec() >= 0.25) {
      traj_base_.points.push_back(pose_state_base_.pose.position);
      traj_arm_.points.push_back(pose_state_arm_.pose.position);

      traj_base_pub_.publish(traj_base_);
      traj_arm_pub_.publish(traj_arm_);

      traj_loop_duration_ = ros::Time::now();
    }

// Publish desired twist of base and arm
    visualize_twistCmd(des_twist_base_, base_frame_, twist_cmd_base_pub_);
    visualize_twistCmd(des_twist_arm_, armbase_frame_, twist_cmd_arm_pub_);

    // Publish direction and pose of desired pose in base_frame and ee_frame
    visualize_poseCmd(pose_cmd_arm_ee_, pose_cmd_arm_dir_pub_,
                      pose_cmd_arm_pub_);
    visualize_poseCmd(pose_cmd_base_, pose_cmd_base_dir_pub_,
                      pose_cmd_base_pub_);

    // Others
    point_eq_error_pub_.publish(distance_eq_error_);
    wrench_e_pub_.publish(wrench_e_msg_);

    pose_eq_pub_.publish(pose_eq_);
    eq_displacement_pub_.publish(eq_displacement_);

    ros::spinOnce();
    loop_rate_.sleep();
  }

}

// Admittance dynamics
void ThingAdmittanceControl::computeAdmittance(Vector6d &des_pose_base,
                                               Vector6d & des_twist_base,
                                               Vector6d &des_pose_arm,
                                               Vector6d & des_twist_arm,
                                               ros::Duration duration) {

// Acceleration of base and arm, error
  Vector6d x_ddot_p, x_ddot_a, delta_pose_arm;

  compute_eqError(x_a_position_, x_a_orientation_, eq_pos_, eq_orientation_,
                  eq_error_);

// Admittance dynamics
// Calculation in base frame
  x_ddot_p = M_p_.inverse()
      * (-D_p_ * (des_twist_base)
          + rotation_base_ * kin_constraints_ * (D_ * x_dot_a_ + K_ * eq_error_)
          + wrench_obs_);

// Calculation in armbase frame
  x_ddot_a = M_a_.inverse()
      * (-(D_ + D_a_) * (des_twist_arm) - K_ * eq_error_ + wrench_e_);

//  ROS_INFO_STREAM("Ang Eq Error: " << eq_error_.tail(3));
//  ROS_INFO_STREAM("Ang Wrench: " << wrench_e_.tail(3));

// Integrate for velocity based interface
  des_twist_base = des_twist_base + x_ddot_p * duration.toSec();
  des_twist_arm = des_twist_arm + x_ddot_a * duration.toSec();

  des_twist_base[2] = 0.0;
  des_twist_base[3] = 0.0;
  des_twist_base[4] = 0.0;

// Integrate for position based interface
  delta_pose_arm = des_twist_arm * duration.toSec();

  des_pose_arm.head(3) = x_a_.head(3) + delta_pose_arm.head(3);
  des_pose_arm.tail(3) = x_a_.tail(3) + delta_pose_arm.tail(3);

  // Since frame origin is equal to base origin, delta is equal to desired
  des_pose_base = des_twist_base * duration.toSec();

}

double ThingAdmittanceControl::quadr_lowpassFilter(double &x, double &f_s,
                                                   double &f_d, Vector3d &X,
                                                   Vector3d &Y) {
  double y, omega_d, alpha, k, b_1, b_2, a_1, a_2;
  // x input, y output, f_s sampling freq, f_d cutoff freq, k gain

  omega_d = 2 * M_PI * f_d / f_s;
  alpha = tan(omega_d / 2);
  k = pow(alpha, 2) / (1.0 + sqrt(2.0) * alpha + pow(alpha, 2));
  a_1 = 2.0 * (pow(alpha, 2) - 1.0) / (1.0 + sqrt(2.0) * alpha + pow(alpha, 2));
  a_2 = (1 - sqrt(2.0) * alpha + pow(alpha, 2))
      / (1 + sqrt(2.0) * alpha + pow(alpha, 2));
  b_1 = 2;
  b_2 = 1;

  X(2) = x;  //store incoming sample in queue

  y = k * X(2) + k * b_1 * X(1) + k * b_2 * X(0) - a_1 * Y(1) - a_2 * Y(0);  //second order lowpass transfer function with Butterworth characteristic

  // Shift back
  X(0) = X(1);
  X(1) = X(2);
  X(2) = 0.0;
  Y(0) = Y(1);
  Y(1) = y;
  Y(2) = 0.0;

  return y;
}

void ThingAdmittanceControl::lin_lowpassFilter(double &x,
                                               std::list<double> & queue,
                                               int &length, double &gain) {
  double sum;
  queue.push_back(x);
  while (queue.size() > length)
    queue.pop_front();

  sum = std::accumulate(queue.begin(), queue.end(), 0.0);
  x = sum * gain / queue.size();

}

// Callbacks
void ThingAdmittanceControl::state_baseCallback(
    const nav_msgs::OdometryConstPtr msg) {

  pose_state_base_.pose = msg->pose.pose;
  pose_state_base_.header = msg->header;

  // Pose of base in frame "odom"
  x_p_position_ << msg->pose.pose.position.x, msg->pose.pose.position.y, msg
      ->pose.pose.position.z;
  x_p_orientation_.coeffs() << msg->pose.pose.orientation.x, msg->pose.pose
      .orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w;
  // Twist of base in frame "odom"
  x_dot_p_ << msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist
      .twist.linear.z, msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg
      ->twist.twist.angular.z;

  // Set time of publishing cmds equal to callback time of base
  pose_cmd_base_.header.stamp = msg->header.stamp;
  pose_cmd_arm_ee_.header.stamp = msg->header.stamp;

}

// Collect data from force sensor (Subscribed to robotiq_force_torque_wrench_zero)
void ThingAdmittanceControl::wrench_inputCallback(
    const geometry_msgs::WrenchStampedConstPtr msg) {

  Vector6d wrench_ft_frame;
  Matrix6d rotation_ft_base_arm;

  // Store wrench input here
  wrench_ft_frame << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force
      .z, msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z;

  // Set rotation_ft_arm_base
  getRotation(rotation_ft_base_arm, listener_ft_, armbase_frame_, ft_frame_);

  // Dead zone for the FT sensor
  if (wrench_ft_frame.topRows(3).norm() < force_dead_zone_thres_) {
    wrench_ft_frame.topRows(3).setZero();
  }
  if (wrench_ft_frame.bottomRows(3).norm() < torque_dead_zone_thres_) {
    wrench_ft_frame.bottomRows(3).setZero();
  }

  // Create Wrench in Arm Base Frame
  wrench_e_
      << (1 - wrench_filter_factor_) * wrench_e_
          + wrench_filter_factor_ * rotation_ft_base_arm * wrench_ft_frame;

  wrench_e_msg_.header.stamp = msg->header.stamp;
  wrench_e_msg_.header.frame_id = armbase_frame_;
  wrench_e_msg_.wrench.force.x = wrench_e_(0);
  wrench_e_msg_.wrench.force.y = wrench_e_(1);
  wrench_e_msg_.wrench.force.z = wrench_e_(2);
  wrench_e_msg_.wrench.torque.x = wrench_e_(3);
  wrench_e_msg_.wrench.torque.y = wrench_e_(4);
  wrench_e_msg_.wrench.torque.z = wrench_e_(5);
}

void ThingAdmittanceControl::local_costmapCallback(
    const nav_msgs::OccupancyGridConstPtr msg) {

  nav_msgs::OccupancyGrid grid;
  grid = *msg;

  int h = 1;  //gradient width
  int N_1 = grid.info.height;  //row size
  int N_2 = grid.info.width;  //column size
  int n_1 = abs(grid.info.height / 2);  //row indicator
  int n_2 = abs(grid.info.width / 2);  //column indicator
  ;  //column indicator
  int n = n_1 * N_2 + n_2;
  int val_n = grid.data[n];
  Vector2d grad;  //gradient
  Vector2d sum;

  grad[0] = -(grid.data[n + h] - grid.data[n - h]) / (2 * h);
  grad[1] = -(grid.data[n + h * N_1] - grid.data[n - h * N_1]) / (2 * h);

  grad = grad.unaryExpr([](double v) {return std::isfinite(v)? v : 0.0;});

  lin_lowpassFilter(grad[0], q_wrench_obs_x_, D_wrench_obs_, P_wrench_obs_);
  lin_lowpassFilter(grad[1], q_wrench_obs_y_, D_wrench_obs_, P_wrench_obs_);

  grad = grad.unaryExpr([](double v) {return std::isfinite(v)? v : 0.0;});

  // Safety
  if (grad.norm() > 30.0) {  // cap of at 30N
    grad = grad.normalized() * 30.0;
  }

  wrench_obs_.head(2) = grad;

  wrench_obs_msg_.header = msg->header;
  wrench_obs_msg_.wrench.force.x = wrench_obs_(0);
  wrench_obs_msg_.wrench.force.y = wrench_obs_(1);
  wrench_obs_msg_.wrench.force.z = wrench_obs_(2);
  wrench_obs_msg_.wrench.torque.x = wrench_obs_(3);
  wrench_obs_msg_.wrench.torque.y = wrench_obs_(4);
  wrench_obs_msg_.wrench.torque.z = wrench_obs_(5);

  wrench_obs_pub_.publish(wrench_obs_msg_);

}

//  Get rotation maxtrix between two frames
bool ThingAdmittanceControl::getRotation(Matrix6d & rotation_matrix,
                                         tf::TransformListener & listener,
                                         std::string target_frame,
                                         std::string source_frame) {

  tf::StampedTransform transform;
  Matrix3d rotation_from_to;

  try {
    //  target_frame  The frame to which data should be transformed
    //  source_frame  The frame where the data originated
    listener.lookupTransform(target_frame, source_frame, ros::Time(0),
                             transform);
    tf::matrixTFToEigen(transform.getBasis(), rotation_from_to);
    rotation_matrix.setZero();
    rotation_matrix.topLeftCorner(3, 3) = rotation_from_to;
    rotation_matrix.bottomRightCorner(3, 3) = rotation_from_to;
  } catch (tf::TransformException ex_) {
    rotation_matrix.setZero();
    ROS_INFO_STREAM(
        "Waiting for TF from " << target_frame << " to " << source_frame);
    return false;
  }

  return true;
}
// Get pose and twist of desired frame
void ThingAdmittanceControl::getFK(Vector3d &position, Quaterniond &rotation,
                                   Vector6d &pose, Vector6d &velocity,
                                   tf::TransformListener & listener,
                                   std::string target_frame,
                                   std::string source_frame) {
//  target_frame  The frame to which data should be transformed
//  source_frame  The frame where the data originated
  tf::StampedTransform transform;
  geometry_msgs::Twist twist;

  try {
    listener.lookupTransform(target_frame, source_frame, ros::Time(0),
                             transform);
    // fill position vector
    position(0) = transform.getOrigin().x();
    position(1) = transform.getOrigin().y();
    position(2) = transform.getOrigin().z();
    // fill rotational vector
    rotation.x() = transform.getRotation().x();
    rotation.y() = transform.getRotation().y();
    rotation.z() = transform.getRotation().z();
    rotation.w() = transform.getRotation().w();
  } catch (tf::TransformException ex) {
    ROS_ERROR("%s", ex.what());
  }

  AngleAxisd x_a_angleaxis(rotation);
  pose.head(3) << position;
  pose.tail(3) << x_a_angleaxis.axis() * x_a_angleaxis.angle();

//tf::quaternionEigenToTF(rotation, pose.getRotation());

  getTwist(twist, listener, target_frame, source_frame);

  velocity(0) = twist.linear.x;
  velocity(1) = twist.linear.y;
  velocity(2) = twist.linear.z;
  velocity(3) = twist.angular.x;
  velocity(4) = twist.angular.y;
  velocity(5) = twist.angular.z;

}
// Get twist between two frames
void ThingAdmittanceControl::getTwist(geometry_msgs::Twist &des_twist,
                                      tf::TransformListener & listener,
                                      std::string obs_frame,
                                      std::string tracking_frame) {

  std::string ref_frame = obs_frame;
  tf::Point ref_point(0, 0, 0);
  std::string ref_point_frame = obs_frame;
  ros::Duration duration(0.1);

  try {
    listener.lookupTwist(tracking_frame, obs_frame, ref_frame, ref_point,
                         ref_point_frame, ros::Time(0), duration, des_twist);
  } catch (tf::TransformException ex) {
    ROS_ERROR("%s", ex.what());
    ros::Duration(0.5).sleep();
  }
}
// Create twist geometry msg and publish
void ThingAdmittanceControl::visualize_twistCmd(Vector6d &twist,
                                                std::string &frame,
                                                ros::Publisher &pub) {

  geometry_msgs::TwistStamped twist_msg;
  twist_msg.header.frame_id = frame;
  twist_msg.header.stamp = ros::Time::now();

  twist_msg.twist.linear.x = twist(0);
  twist_msg.twist.linear.y = twist(1);
  twist_msg.twist.linear.z = twist(2);
  twist_msg.twist.angular.x = twist(3);
  twist_msg.twist.angular.y = twist(4);
  twist_msg.twist.angular.z = twist(5);

  pub.publish(twist_msg);
}
// Create polygon msg and publish together with original msg
void ThingAdmittanceControl::visualize_poseCmd(
    geometry_msgs::PoseStamped &pose_direction, ros::Publisher &pub_dir,
    ros::Publisher &pub_cmd) {

  geometry_msgs::PolygonStamped pose_cmd_arm_dir;  //Direction of Arm Pose Command
  geometry_msgs::Point32 origin;
  geometry_msgs::Point32 direction;
  float scale = 100.0;

  origin.x = 0.0;
  origin.y = 0.0;
  origin.z = 0.0;
  direction.x = 100 * pose_direction.pose.position.x;
  direction.y = 100 * pose_direction.pose.position.y;
  direction.z = 100 * pose_direction.pose.position.z;

  pose_cmd_arm_dir.header.frame_id = pose_direction.header.frame_id;
  pose_cmd_arm_dir.polygon.points.push_back(origin);
  pose_cmd_arm_dir.polygon.points.push_back(direction);

  pub_dir.publish(pose_cmd_arm_dir);  // Publish cmd direction
  pub_cmd.publish(pose_direction);  // Publish cmd pose

}
// Convert Vector6d to pose msg
void ThingAdmittanceControl::convert_to_poseMsg(
    geometry_msgs::PoseStamped &pose_msg, Vector6d &pose, std::string frame) {

  Vector3d rot_vec;  // Rotational Vector
  rot_vec.setZero();
  rot_vec = pose.bottomRows(3);

  AngleAxisd::Scalar theta(rot_vec.norm());  // Transform to proper angle axis
  if (rot_vec.norm() > 0)
    rot_vec.normalize();     // Normalize for axis
  AngleAxisd axis(theta, rot_vec);
  Quaterniond orient(axis);  //Transform to Quaternion

  pose_msg.header.frame_id = frame;
  pose_msg.pose.position.x = pose(0);
  pose_msg.pose.position.y = pose(1);
  pose_msg.pose.position.z = pose(2);
  pose_msg.pose.orientation.x = orient.x();
  pose_msg.pose.orientation.y = orient.y();
  pose_msg.pose.orientation.z = orient.z();
  pose_msg.pose.orientation.w = orient.w();

}
// Convert and transform a Vector6d pose to a pose msg
void ThingAdmittanceControl::transformPose(geometry_msgs::PoseStamped &pose_cmd,
                                           Vector6d &pose,
                                           tf::TransformListener & listener,
                                           std::string target_frame,
                                           std::string source_frame) {
// Input 6d pose vector in source frame
// Output geometry msg in target frame

  geometry_msgs::PoseStamped pose_input;
// Rotational Vector
  Vector3d rot_vec;
  rot_vec.setZero();
  rot_vec(0) = pose(3);
  rot_vec(1) = pose(4);
  rot_vec(2) = pose(5);

  AngleAxisd::Scalar theta(rot_vec.norm());  // Transform to proper angle axis
  rot_vec.normalize();  // Normalize for axis
  AngleAxisd axis(theta, rot_vec);
  Quaterniond orient(axis);  //Transform to Quaternion

  pose_input.header.frame_id = source_frame;
  pose_input.pose.position.x = pose(0);
  pose_input.pose.position.y = pose(1);
  pose_input.pose.position.z = pose(2);
  pose_input.pose.orientation.x = orient.x();
  pose_input.pose.orientation.y = orient.y();
  pose_input.pose.orientation.z = orient.z();
  pose_input.pose.orientation.w = orient.w();

  listener.transformPose(target_frame, pose_input, pose_cmd);
}

// Input geometry msgs for base and arm
// Output 10d float vector (3d pos arm, 4d quaternion arm, 2d pos base, 1d rot base
void ThingAdmittanceControl::publish_poseCmd(
    geometry_msgs::PoseStamped &pose_base, geometry_msgs::PoseStamped &pose_arm,
    Vector6d &pose_des_base) {

  Quaterniond base_quat;
  base_quat.x() = pose_base.pose.orientation.x;
  base_quat.y() = pose_base.pose.orientation.y;
  base_quat.z() = pose_base.pose.orientation.z;
  base_quat.w() = pose_base.pose.orientation.w;

  Vector3d base_rot;

  // Like this?
//  base_rot = base_quat.normalized().toRotationMatrix().eulerAngles(2,0,2);
  // Or this?
  AngleAxisd base_angleaxis(base_quat);
  base_rot << base_angleaxis.axis() * base_angleaxis.angle();

  std_msgs::Float64MultiArray pose_msg, pose_des_base_msg;
  pose_msg.data.resize(10);
  pose_des_base_msg.data.resize(6);

  pose_msg.data[0] = pose_arm.pose.position.x;
  pose_msg.data[1] = pose_arm.pose.position.y;
  pose_msg.data[2] = pose_arm.pose.position.z;
  pose_msg.data[3] = pose_arm.pose.orientation.x;
  pose_msg.data[4] = pose_arm.pose.orientation.y;
  pose_msg.data[5] = pose_arm.pose.orientation.z;
  pose_msg.data[6] = pose_arm.pose.orientation.w;
  pose_msg.data[7] = pose_base.pose.position.x;
  pose_msg.data[8] = pose_base.pose.position.y;
  pose_msg.data[9] = base_rot(2);

  // Publish msg for thing control
  if (pose_output_)
    pose_cmd_pub_.publish(pose_msg);
}

// Compute delta pose between arm position and equilibrium point in armbase frame
void ThingAdmittanceControl::compute_eqError(Vector3d &x_a_position,
                                             Quaterniond &x_a_orientation,
                                             Vector3d &eq_position,
                                             Quaterniond &eq_orientation,
                                             Vector6d &eq_error) {

// Translation error
  eq_error.topRows(3) = x_a_position - eq_position;

// Orientation error
// Invert if negative
  if (eq_orientation.coeffs().dot(x_a_orientation.coeffs()) < 0.0) {
    x_a_orientation.coeffs() << -x_a_orientation.coeffs();
  }

  Quaterniond quat_rot_err(x_a_orientation * eq_orientation.inverse());

// Normalize error quaternion
  if (quat_rot_err.coeffs().norm() > 1e-3) {
    quat_rot_err.coeffs()
        << quat_rot_err.coeffs() / quat_rot_err.coeffs().norm();
  }
// Create angle axis
  AngleAxisd err_arm_des_orient(quat_rot_err);

// Rotational vector of angle axis (phi = theta * n)
  eq_error.bottomRows(3)
      << err_arm_des_orient.axis() * err_arm_des_orient.angle();

  geometry_msgs::Vector3Stamped vec1, vec2;
  vec1.header.frame_id = armbase_frame_;
  vec1.vector.x = -eq_error(0);
  vec1.vector.y = -eq_error(1);
  vec1.vector.z = -eq_error(2);
  listener_arm_.transformVector(ee_frame_, vec1, vec2);

  distance_eq_error_.point.x = vec2.vector.x;
  distance_eq_error_.point.y = vec2.vector.y;
  distance_eq_error_.point.z = vec2.vector.z;
}

//////////////////////
/// INITIALIZATION ///
//////////////////////
void ThingAdmittanceControl::initTF() {
  tf::TransformListener listener;
  tf::StampedTransform transform;
  Matrix6d rot_matrix;
  rotation_base_.setZero();

  // Makes sure all TFs exists before enabling transformations in the callbacks
  while (!getRotation(rot_matrix, listener, world_frame_, base_frame_)) {
    sleep(1);
  }
  base_world_ready_ = true;
  while (!getRotation(rotation_base_, listener, base_frame_, armbase_frame_)) {
    sleep(1);
  }
  while (!getRotation(rot_matrix, listener, armbase_frame_, base_frame_)) {
    sleep(1);
  }
  while (!getRotation(rot_matrix, listener, armbase_frame_, world_frame_)) {
    sleep(1);
  }
  world_armbase_ready_ = true;
  while (!getRotation(rot_matrix, listener, armbase_frame_, ee_frame_)) {
    sleep(1);
  }
  while (!getRotation(rot_matrix, listener, armbase_frame_, ft_frame_)) {
    sleep(1);
  }
  ft_armbase_ready_ = true;
}

void ThingAdmittanceControl::initMarker() {

  // Create arrow marker for displacement of eq position
  geometry_msgs::Point eq_pos_init;
  eq_pos_init.x = eq_pos_(0);
  eq_pos_init.y = eq_pos_(1);
  eq_pos_init.z = eq_pos_(2);

  eq_displacement_.type = 0;
  eq_displacement_.header.frame_id = armbase_frame_;
  eq_displacement_.header.stamp = ros::Time();
  eq_displacement_.color.r = 0.5;
  eq_displacement_.color.g = 0.8;
  eq_displacement_.color.b = 0.45;
  eq_displacement_.color.a = 1;

  eq_displacement_.scale.x = 0.01;
  eq_displacement_.scale.y = 0.05;
  eq_displacement_.points.push_back(eq_pos_init);
  eq_displacement_.points.push_back(eq_pos_init);

  // Base Trajectory Marker
  traj_base_.header.frame_id = world_frame_;
  traj_base_.header.stamp = ros::Time::now();
  traj_base_.type = 4;
  traj_base_.scale.x = 0.005;
  traj_base_.color.r = 1;
  traj_base_.color.g = 0;
  traj_base_.color.b = 1;
  traj_base_.color.a = 1;

  // Arm Trajectory Marker
  traj_arm_.header.frame_id = armbase_frame_;
  traj_arm_.header.stamp = ros::Time::now();
  traj_arm_.type = 4;
  traj_arm_.scale.x = 0.005;
  traj_arm_.color.r = 1;
  traj_arm_.color.g = 0;
  traj_arm_.color.b = 1;
  traj_arm_.color.a = 1;

}

} /* namespace */

