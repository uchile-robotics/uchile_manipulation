// ROS
#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <eigen_conversions/eigen_msg.h>
#include "hb_grasp_generator/grasp_generator.h"

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "grasp_generator_test");

  ROS_INFO_STREAM_NAMED("main","Simple Grasps Test");

  ros::AsyncSpinner spinner(2);
  spinner.start();

  // Benchmark time
  ros::Time start_time;
  start_time = ros::Time::now();

  // Load visualization tools for publishing to Rviz
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene;
  planning_scene.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_(new moveit_visual_tools::MoveItVisualTools("bender/base_link", "/grasp_test", planning_scene));
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
  const robot_model::JointModelGroup* ee_jmg = visual_tools_->getRobotModel()->getJointModelGroup("l_gripper");
  for (std::size_t i = 0; i < possible_grasps.size(); ++i)
  {
    visual_tools_->publishEEMarkers(possible_grasps[i].grasp_pose.pose ,ee_jmg, rviz_visual_tools::DARK_GREY);
    visual_tools_->triggerBatchPublish();
    ros::Duration(0.03).sleep();
  }

  // Benchmark time
  double duration = (ros::Time::now() - start_time).toNSec() * 1e-6;
  ROS_INFO_STREAM_NAMED("","Total time: " << duration);

  return 0;
}
