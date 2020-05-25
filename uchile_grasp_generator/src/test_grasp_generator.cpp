// ROS
#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
// Eigen
#include <eigen_conversions/eigen_msg.h>

#include "uchile_grasp_generator/grasp_generator.h"

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "grasp_generator_test");

  ROS_INFO_STREAM_NAMED("main", "Simple Grasps Test");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Benchmark time
  ros::Time start_time;
  start_time = ros::Time::now();

  // Load visualization tools for publishing to Rviz
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene;
  planning_scene.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));
  moveit_visual_tools::MoveItVisualToolsPtr
      visual_tools_(new moveit_visual_tools::MoveItVisualTools("bender/base_link", "/grasp_test", planning_scene));
  visual_tools_->setLifetime(120.0);
  visual_tools_->loadMarkerPub();

  // Load grasp generator
  ros::NodeHandle nh("~");
  uchile_grasp_generator::GraspOptions opt;
  opt.load(nh, "l_arm");
  ros::NodeHandle grasp_nh(nh, "l_arm");
  uchile_grasp_generator::CylindricalGraspGeneratorPtr simple_grasps_(new uchile_grasp_generator::CylindricalGraspGenerator(grasp_nh, opt));
  // Test object pose
  geometry_msgs::Pose object_pose;
  object_pose.position.x = 0.50;
  object_pose.position.y = 0.29;
  object_pose.position.z = 0.60;

  object_pose.orientation.x = 0.0;
  object_pose.orientation.y = 0.0;
  object_pose.orientation.z = 0.0;
  object_pose.orientation.w = 1.0;

  // Grasp vector
  std::vector<moveit_msgs::Grasp> possible_grasps;

  // Generate set of grasps for one object
  simple_grasps_->generateGrasp(object_pose, possible_grasps);
  ROS_INFO_STREAM("Generated grasps: " << possible_grasps.size());

  // Visualize grasps
  ros::Duration(2.0).sleep(); // Wait for RViz
  visual_tools_->enableBatchPublishing(true);
  visual_tools_->publishCylinder(object_pose, rviz_visual_tools::BLUE, 0.05, 0.05);
  visual_tools_->trigger();
  const robot_model::JointModelGroup *ee_jmg = visual_tools_->getRobotModel()->getJointModelGroup("l_gripper");
  Eigen::Isometry3d grasp_pose, pregrasp_pose, arrow_pose;
  Eigen::Vector3d approach;
  Eigen::Quaterniond quat;
  // Hardcoded way, publishAnimatedGrasps didn't work
  for (std::size_t i = 0; i < possible_grasps.size(); ++i)
  {
    // View pregrasp pose
    geometry_msgs::PoseStamped
        pregrasp = uchile_grasp_generator::getPreGraspPose(possible_grasps[i], "bender/l_wrist_pitch_link");
    visual_tools_->publishEEMarkers(pregrasp.pose, ee_jmg, rviz_visual_tools::DARK_GREY);
    visual_tools_->trigger();
    ros::Duration(0.01).sleep();

    // Convert to Eigen
    tf::poseMsgToEigen(pregrasp.pose, pregrasp_pose);
    tf::poseMsgToEigen(possible_grasps[i].grasp_pose.pose, grasp_pose);
    // Get approach vector wrt global frame
    approach = grasp_pose.rotation() * Eigen::Vector3d(
        possible_grasps[i].pre_grasp_approach.direction.vector.x * possible_grasps[i].pre_grasp_approach.desired_distance,
        possible_grasps[i].pre_grasp_approach.direction.vector.y * possible_grasps[i].pre_grasp_approach.desired_distance,
        possible_grasps[i].pre_grasp_approach.direction.vector.z * possible_grasps[i].pre_grasp_approach.desired_distance
    );
    // Get quaternion that transform x axis of pregrasp pose to global x axis
    quat.setFromTwoVectors(Eigen::Vector3d::UnitX(), approach);
    arrow_pose = quat; // Copy rotation from quaternion
    arrow_pose.translation() = pregrasp_pose.translation(); // Copy translation of pregrasp pose
    visual_tools_->publishXArrow(arrow_pose,
                                rviz_visual_tools::RED,
                                rviz_visual_tools::MEDIUM,
                                possible_grasps[i].pre_grasp_approach.desired_distance);
    visual_tools_->trigger();
    ros::Duration(0.01).sleep();

    // View grasp pose
    visual_tools_->publishEEMarkers(possible_grasps[i].grasp_pose.pose, ee_jmg, rviz_visual_tools::DARK_GREY);
    visual_tools_->trigger();
    ros::Duration(0.01).sleep();

    if (!ros::ok())
      break;
  }
  // Benchmark time
  ROS_INFO_STREAM_NAMED("", "Total time: " << (ros::Time::now() - start_time).toSec());

  return 0;
}
