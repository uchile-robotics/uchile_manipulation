#ifndef UCHILE_GRASP_GRASP_OPTIONS_H
#define UCHILE_GRASP_GRASP_OPTIONS_H

#include <ostream>
// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
// ROS
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>

namespace uchile_grasp_generator
{

class GraspOptions
{
 public:
  // Grasp point to end effector
  Eigen::Affine3d grasp_pose_to_eff;
  // Frames
  std::string base_frame;
  std::string end_effector_name;
  std::string end_effector_parent_link;
  std::vector<std::string> joint_names;
  std::vector<double> grasp_posture;
  std::vector<double> pre_grasp_posture;
  trajectory_msgs::JointTrajectory grasp_posture_msg;
  trajectory_msgs::JointTrajectory pre_grasp_posture_msg;
  double pregrasp_time_from_start;
  double grasp_time_from_start;

 public:
  GraspOptions();

  bool load(const ros::NodeHandle &nh, const std::string &end_effector);

  friend std::ostream &operator<<(std::ostream &os, const GraspOptions &opt);
};

class CylindricalGraspGeneratorOptions
{
 private:
  const std::string name_;
 public:
  // Yaw params
  int yaw_angle_count;
  // Pitch params
  int pitch_angle_count;
  double pitch_angle_min;
  double pitch_angle_max;
  double pitch_angle_res;
  // Roll params
  int roll_angle_count;
  double roll_angle_min;
  double roll_angle_max;
  double roll_angle_res;
  // Pregrasp params
  int pregrasp_count;
  double pregrasp_min;
  double pregrasp_max;
  double pregrasp_res;
  double approach_angle;

 public:
  CylindricalGraspGeneratorOptions();

  bool load(const ros::NodeHandle &nh);

  friend std::ostream &operator<<(std::ostream &os, const CylindricalGraspGeneratorOptions &opt);
};
}

#endif // UCHILE_GRASP_GRASP_OPTIONS_H
