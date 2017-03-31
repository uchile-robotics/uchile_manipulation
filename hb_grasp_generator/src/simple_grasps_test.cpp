// ROS
#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
// Eigen
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "hb_grasp_generator/grasp_generator.h"

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
  hb_grasp_generator::CylindricalGraspGeneratorPtr simple_grasps_(new hb_grasp_generator::CylindricalGraspGenerator());

  // Object pose
  geometry_msgs::Pose object_pose;
  object_pose.position.x = 0.0;
  object_pose.position.y = 0.0;
  object_pose.position.z = 0.0;
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
  visual_tools_->triggerBatchPublish();
  const robot_model::JointModelGroup *ee_jmg = visual_tools_->getRobotModel()->getJointModelGroup("l_gripper");
  Eigen::Affine3d grasp_pose, pregrasp_pose;
  Eigen::Quaterniond quat;
  // Hardcoded way, publishAnimatedGrasps didn't work
  for (std::size_t i = 0; i < possible_grasps.size(); ++i)
  {
    // View pregrasp pose
    geometry_msgs::PoseStamped
        pregrasp = hb_grasp_generator::getPreGraspPose(possible_grasps[i], "bender/l_wrist_pitch_link");
    visual_tools_->publishEEMarkers(pregrasp.pose, ee_jmg, rviz_visual_tools::DARK_GREY);
    visual_tools_->triggerBatchPublish();
    ros::Duration(0.06).sleep();

    // View aproximation vector
    tf::poseMsgToEigen(pregrasp.pose, pregrasp_pose);
    tf::poseMsgToEigen(possible_grasps[i].grasp_pose.pose, grasp_pose);
    quat.setFromTwoVectors(pregrasp_pose.translation(), grasp_pose.translation()); // From pregrasp to grasp
    visual_tools_->publishZArrow(
        pregrasp_pose * quat * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()), // Turn around x axis
        rviz_visual_tools::RED,
        rviz_visual_tools::REGULAR,
        possible_grasps[i].pre_grasp_approach.desired_distance);
    visual_tools_->triggerBatchPublish();
    ros::Duration(0.03).sleep();

    // View grasp pose
    visual_tools_->publishEEMarkers(possible_grasps[i].grasp_pose.pose, ee_jmg, rviz_visual_tools::DARK_GREY);
    visual_tools_->triggerBatchPublish();
    ros::Duration(0.03).sleep();
  }
  // Benchmark time
  double duration = (ros::Time::now() - start_time).toNSec() * 1e-6;
  ROS_INFO_STREAM_NAMED("", "Total time: " << duration);

  return 0;
}
