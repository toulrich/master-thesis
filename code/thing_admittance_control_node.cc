/*
 * thing_admittance_control_node.cc
 *
 *  Created on: Mar 9, 2018
 *      Author: ulrichto
 */

#include <ros/ros.h>
#include "thing_admittance_control/ThingAdmittanceControl.h"

int main(int argc, char** argv) {

  ros::init(argc, argv, "thing_admittance_control");
  ros::NodeHandle nodeHandle("~");
  double loop_rate;  // [Hz]

  //INPUT PARAMETERS (PUBLIC)
  std::vector<double> M_p, M_a, D, D_p, D_a, K, eq_pose;
  double wrench_filter_factor, force_dead_zone_thres, torque_dead_zone_thres, P_base_pos, P_arm_pos, P_wrench_obs;
  int D_wrench_obs;
  bool pose_output;

  // Rostopics
  std::string wrench_input_topic = "/robotiq_force_torque_wrench_zero";
  std::string wrench_control_topic = "wrench_control";
  std::string platform_state_topic = "/odometry/filtered";
  std::string pose_command_topic = "/servo/command";
  std::string pose_platform_topic = "pose_cmd_rb";
  std::string pose_arm_topic = "pose_cmd_ur10";
  std::string pose_eq_topic = "pose_eq_current";
  std::string eq_error_topic = "distance_to_eq_point";
  std::string twist_platform_topic = "twist_cmd_rb";
  std::string twist_arm_topic = "twist_cmd_ur10";

  // Frames
  std::string ee_frame, ft_frame, armbase_frame, platform_frame, world_frame;

  if (!nodeHandle.getParam("frequency", loop_rate)) {
    ROS_ERROR("Couldn't retrieve the loop frequency.");
    return -1;
  }

  // Admittance
  if (!nodeHandle.getParam("mass_platform", M_p)) {
    ROS_ERROR("Couldn't retrieve mass platform.");
    return -1;
  }

  if (!nodeHandle.getParam("mass_arm", M_a)) {
    ROS_ERROR("Couldn't retrieve mass of the arm.");
    return -1;
  }

  if (!nodeHandle.getParam("damping_coupling", D)) {
    ROS_ERROR("Couldn't retrieve damping of the coupling.");
    return -1;
  }

  if (!nodeHandle.getParam("damping_platform", D_p)) {
    ROS_ERROR("Couldn't retrieve damping of the platform.");
    return -1;
  }

  if (!nodeHandle.getParam("damping_arm", D_a)) {
    ROS_ERROR("Couldn't retrieve damping of the arm.");
    return -1;
  }

  if (!nodeHandle.getParam("stiffness_coupling", K)) {
    ROS_ERROR("Couldn't retrieve stiffness of the coupling.");
    return -1;
  }
  if (!nodeHandle.getParam("equilibrium_point_spring", eq_pose)) {
    ROS_ERROR("Couldn't retrieve equilibrium of the spring.");
    return -1;
  }

  if (!nodeHandle.getParam("wrench_filter_factor", wrench_filter_factor)) {
    ROS_ERROR("Couldn't retrieve wrench filter factor.");
    return -1;
  }

  if (!nodeHandle.getParam("force_dead_zone_thres", force_dead_zone_thres)) {
    ROS_ERROR("Couldn't retrieve force_dead_zone threshold.");
    return -1;
  }

  if (!nodeHandle.getParam("torque_dead_zone_thres", torque_dead_zone_thres)) {
    ROS_ERROR("Couldn't retrieve torque_dead_zone threshold. ");
    return -1;
  }

  // Other
  if (!nodeHandle.getParam("gain_base_pose", P_base_pos)) {
    ROS_ERROR("Couldn't retrieve gain_base_pose. ");
    return -1;
  }

  if (!nodeHandle.getParam("gain_arm_pose", P_arm_pos)) {
    ROS_ERROR("Couldn't retrieve gain_arm_pose. ");
    return -1;
  }

  if (!nodeHandle.getParam("gain_wrench_obstacle", P_wrench_obs)) {
    ROS_ERROR("Couldn't retrieve gain_wrench_obstacle. ");
    return -1;
  }

  if (!nodeHandle.getParam("damping_wrench_obstacle", D_wrench_obs)) {
    ROS_ERROR("Couldn't retrieve damping_wrench_obstacle. ");
    return -1;
  }


  if (!nodeHandle.getParam("pose_output", pose_output)) {
    ROS_ERROR("Couldn't retrieve the pose output flag.");
    return -1;
  }

  // Frames
  if (!nodeHandle.getParam("frame/ee", ee_frame)) {
    ROS_ERROR("Couldn't retrieve the ee frame.");
    return -1;
  }

  if (!nodeHandle.getParam("frame/ft", ft_frame)) {
    ROS_ERROR("Couldn't retrieve the force torque sensor frame.");
    return -1;
  }

  if (!nodeHandle.getParam("frame/armbase", armbase_frame)) {
    ROS_ERROR("Couldn't retrieve the armbase frame.");
    return -1;
  }

  if (!nodeHandle.getParam("frame/platform", platform_frame)) {
    ROS_ERROR("Couldn't retrieve the platform frame.");
    return -1;
  }

  if (!nodeHandle.getParam("frame/world", world_frame)) {
    ROS_ERROR("Couldn't retrieve the world frame.");
    return -1;
  }

  thing_admittance_control::ThingAdmittanceControl thingAdmittanceControl(
      nodeHandle, wrench_input_topic, wrench_control_topic,
      platform_state_topic, pose_command_topic, pose_platform_topic,
      pose_arm_topic, pose_eq_topic, eq_error_topic, twist_platform_topic,
      twist_arm_topic, loop_rate, pose_output, ee_frame,
      ft_frame, armbase_frame, platform_frame, world_frame, M_p, M_a, D, D_p,
      D_a, K, eq_pose, wrench_filter_factor, force_dead_zone_thres,
      torque_dead_zone_thres, P_base_pos, P_arm_pos, P_wrench_obs, D_wrench_obs);

  ROS_INFO("Constructor set up");

  thingAdmittanceControl.execute();

  return 0;
}

