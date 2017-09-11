#include <vector>
#include "uchile_grasp_generator/grasp_options.h"

namespace uchile_grasp_generator
{

GraspOptions::GraspOptions():
    base_frame("base_link"),
    pregrasp_time_from_start(0.0),
    grasp_time_from_start(0.0)
{}

bool GraspOptions::load(const ros::NodeHandle &nh, const std::string &end_effector)
{
  // End effector nodehandle
  ros::NodeHandle ee_nh(nh, end_effector);

  // Load base_link a param
  if (!ee_nh.hasParam("base_frame"))
  {
    ROS_ERROR_STREAM_NAMED("grasp_options_loader",
                           "Grasp configuration parameter \"base_frame\" missing from parameter server. Searching in namespace: "
                               << ee_nh.getNamespace());
    return false;
  }
  ee_nh.getParam("base_frame", base_frame);


  // Load a param "pregrasp_time_from_start"
  if (!ee_nh.hasParam("pregrasp_time_from_start"))
  {
    ROS_ERROR_STREAM_NAMED("grasp_options_loader",
                           "Grasp configuration parameter \"pregrasp_time_from_start\" missing from parameter server. Searching in namespace: "
                               << ee_nh.getNamespace());
    return false;
  }
  ee_nh.getParam("pregrasp_time_from_start", pregrasp_time_from_start);

  // Load a param "grasp_time_from_start"
  if (!ee_nh.hasParam("grasp_time_from_start"))
  {
    ROS_ERROR_STREAM_NAMED("grasp_options_loader",
                           "Grasp configuration parameter \"grasp_time_from_start\" missing from parameter server. Searching in namespace: "
                               << ee_nh.getNamespace());
    return false;
  }
  ee_nh.getParam("grasp_time_from_start", grasp_time_from_start);

  // Load a param "end_effector_name"
  if (!ee_nh.hasParam("end_effector_name"))
  {
    ROS_ERROR_STREAM_NAMED("grasp_options_loader",
                           "Grasp configuration parameter \"end_effector_name\" missing from parameter server. Searching in namespace: "
                               << ee_nh.getNamespace());
    return false;
  }
  ee_nh.getParam("end_effector_name", end_effector_name);

  // Load a param "end_effector_parent_link"
  if (!ee_nh.hasParam("end_effector_parent_link"))
  {
    ROS_ERROR_STREAM_NAMED("grasp_options_loader",
                           "Grasp configuration parameter \"end_effector_parent_link\" missing from parameter server. Searching in namespace: "
                               << ee_nh.getNamespace());
    return false;
  }
  ee_nh.getParam("end_effector_parent_link", end_effector_parent_link);

  // Check for joints params
  if (!ee_nh.hasParam("joints"))
  {
    ROS_ERROR_STREAM_NAMED("grasp_options_loader",
                           "Grasp configuration parameter \"joints\" missing from parameter server. Searching in namespace: "
                               << ee_nh.getNamespace());
    return false;
  }
  // Get joint names
  ee_nh.getParam("joints", joint_names);

  // Check for grasp_posture
  if (!ee_nh.hasParam("grasp_posture"))
  {
    ROS_ERROR_STREAM_NAMED("grasp_options_loader",
                           "Grasp configuration parameter \"grasp_posture\" missing from parameter server. Searching in namespace: "
                               << ee_nh.getNamespace());
    return false;
  }
  // Get grasp_posture
  ee_nh.getParam("grasp_posture", grasp_posture);
  // Check grasp_posture lenght
  if (joint_names.size() != grasp_posture.size())
  {
    ROS_ERROR_STREAM_NAMED("grasp_options_loader",
                           "Parameter length of \"grasp_posture\" must be the same that \"joint_names\".");
    return false;
  }

  // Check for pregrasp_posture
  if (!ee_nh.hasParam("pregrasp_posture"))
  {
    ROS_ERROR_STREAM_NAMED("grasp_options_loader",
                           "Grasp configuration parameter \"pregrasp_posture\" missing from parameter server. Searching in namespace: "
                               << ee_nh.getNamespace());
    return false;
  }
  // Get pregrasp_posture
  ee_nh.getParam("pregrasp_posture", pre_grasp_posture);
  // Check pregrasp_posture lenght
  if (joint_names.size() != pre_grasp_posture.size())
  {
    ROS_ERROR_STREAM_NAMED("grasp_options_loader",
                           "Parameter length of \"pregrasp_posture\" must be the same that \"joint_names\".");
    return false;
  }

  // --------------------------------------------------------
  // Grasp pose to end effector
  std::vector<double> pose_elements;
  if (!ee_nh.hasParam("grasp_pose_to_eef"))
  {
    ROS_WARN("Parameter \"grasp_pose_to_eef\" missing. Using default.");
    pose_elements.resize(6, 0.0); // Empty transform
  }
  ee_nh.getParam("grasp_pose_to_eef", pose_elements);
  // Check size
  if (pose_elements.size() != 6)
  {
    ROS_ERROR("Parameter \"grasp_pose_to_eef\" must have 6 components [x, y, z, roll, pitch, yaw].");
    return false;
  }
  // Fill transform
  grasp_pose_to_eff = Eigen::Affine3d::Identity() * Eigen::AngleAxisd(pose_elements[3], Eigen::Vector3d::UnitX())
      * Eigen::AngleAxisd(pose_elements[4], Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(pose_elements[5], Eigen::Vector3d::UnitZ());
  grasp_pose_to_eff.translation().x() = pose_elements[0];
  grasp_pose_to_eff.translation().y() = pose_elements[1];
  grasp_pose_to_eff.translation().z() = pose_elements[2];

  // -------------------------------
  // Create pre grasp posture
  pre_grasp_posture_msg.header.frame_id = base_frame;
  pre_grasp_posture_msg.header.stamp = ros::Time::now();
  // Name of joints:
  pre_grasp_posture_msg.joint_names = joint_names;
  // Position of joints
  pre_grasp_posture_msg.points.resize(1);
  pre_grasp_posture_msg.points[0].positions = pre_grasp_posture;
  pre_grasp_posture_msg.points[0].time_from_start = ros::Duration(pregrasp_time_from_start);
  // -------------------------------
  // Create grasp posture
  grasp_posture_msg.header.frame_id = base_frame;
  grasp_posture_msg.header.stamp = ros::Time::now();
  // Name of joints:
  grasp_posture_msg.joint_names = joint_names;
  // Position of joints
  grasp_posture_msg.points.resize(1);
  grasp_posture_msg.points[0].positions = grasp_posture;
  grasp_posture_msg.points[0].time_from_start = ros::Duration(grasp_time_from_start);
}

std::ostream &operator<<(std::ostream &os, const uchile_grasp_generator::GraspOptions &opt)
{
  os << "Grasp generator options:" << std::endl;
  os << "Base link: " << opt.base_frame << std::endl;
  os << "End effector: " << opt.end_effector_name << std::endl;
  os << "End effector parent: " << opt.end_effector_parent_link << std::endl;

  os << "Joint names: ";
  for (std::size_t i = 0; i < opt.joint_names.size() - 1; ++i) os << opt.joint_names[i] << ", ";
  os << opt.joint_names[opt.joint_names.size() - 1] << std::endl;

  os << "Grasp posture: ";
  for (std::size_t i = 0; i < opt.grasp_posture.size() - 1; ++i) os << opt.grasp_posture[i] << ", ";
  os << opt.grasp_posture[opt.grasp_posture.size() - 1] << std::endl;

  os << "Pregrasp posture: ";
  for (std::size_t i = 0; i < opt.pre_grasp_posture.size() - 1; ++i) os << opt.pre_grasp_posture[i] << ", ";
  os << opt.pre_grasp_posture[opt.pre_grasp_posture.size() - 1];

  return os;
}


CylindricalGraspGeneratorOptions::CylindricalGraspGeneratorOptions():
  name_("cylindrical_grasp_generator"),
  // Default values
  yaw_angle_count(50),
  pitch_angle_count(5),
  pitch_angle_min(-M_PI / 180.0 * 15.0), // 15 degrees
  pitch_angle_max(M_PI / 180.0 * 15.0),
  pitch_angle_res((pitch_angle_max - pitch_angle_min) / pitch_angle_count),
  roll_angle_count(5),
  roll_angle_min(-M_PI / 180.0 * 5.0), // 5 degrees
  roll_angle_max(M_PI / 180.0 * 5.0),
  roll_angle_res((roll_angle_max - roll_angle_min) / roll_angle_count),
  pregrasp_count(5),
  pregrasp_min(0.1),
  pregrasp_max(0.3),
  pregrasp_res((pregrasp_max - pregrasp_min) / pregrasp_count),
  approach_angle(0.0)
{}

bool CylindricalGraspGeneratorOptions::load(const ros::NodeHandle &nh)
{
  ROS_DEBUG_STREAM_NAMED(name_, "Searching in namespace: " << nh.getNamespace());
  // --------------------------------------------------------
  // Grasp data
  // Approach angle
  nh.param<double>("approach_angle", approach_angle, 0.0);
  // Yaw angle: 50 points
  nh.param<int>("yaw_angle_count", yaw_angle_count, 50);

  // Pitch angle
  // 5 points
  nh.param<int>("pitch_angle_count", pitch_angle_count, 5);
  // 15 degrees
  nh.param<double>("pitch_angle_min", pitch_angle_min, -M_PI / 180.0 * 15.0);
  nh.param<double>("pitch_angle_max", pitch_angle_max, M_PI / 180.0 * 15.0);
  pitch_angle_res = (pitch_angle_max - pitch_angle_min) / pitch_angle_count;

  // Roll angle
  // 5 points
  nh.param<int>("roll_angle_count", roll_angle_count, 5);
  // 5 degrees
  nh.param<double>("roll_angle_min", roll_angle_min, -M_PI / 180.0 * 5.0);
  nh.param<double>("roll_angle_max", roll_angle_max, M_PI / 180.0 * 5.0);
  roll_angle_res = (roll_angle_max - roll_angle_min) / roll_angle_count;

  // --------------------------------------------------------
  // Pregrasp data
  // 3 points
  nh.param<int>("pregrasp_count", pregrasp_count, 5);
  // 5 degrees
  nh.param<double>("pregrasp_min", pregrasp_min, 0.1);
  nh.param<double>("pregrasp_max", pregrasp_max, 0.3);
  pregrasp_res = (pregrasp_max - pregrasp_min) / pregrasp_count;
}

std::ostream &operator<<(std::ostream &os, const uchile_grasp_generator::CylindricalGraspGeneratorOptions &opt)
{
  os << "Cylindrical grasp generator options:" << std::endl;
  os << "Yaw angle: count: " << opt.yaw_angle_count << std::endl;
  os << "Pitch angle: count: " << opt.pitch_angle_count << " (" << opt.pitch_angle_min
     << ", " << opt.pitch_angle_min << ")" << std::endl;
  os << "Roll angle: count: " << opt.roll_angle_count << " (" << opt.roll_angle_min
     << ", " << opt.roll_angle_max << ")" << std::endl;
  os << "Pregrasp options:" << std::endl;
  os << "Pregrasp count: " << opt.pregrasp_count << " (" << opt.pregrasp_min << ", "
     << opt.pregrasp_max << ")";
  return os;
}
}
