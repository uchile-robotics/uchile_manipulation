// ROS
#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/JointState.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

// MoveIt
#include <moveit_msgs/MoveGroupAction.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

// Rviz
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// Grasp
#include <moveit_visual_tools/moveit_visual_tools.h>

// Grasp generation
#include "hb_grasp_generator/grasp_options.h"
#include "hb_grasp_generator/grasp_filter.h"
#include "hb_grasp_generator/grasp_generator.h"

namespace hb_grasp_generator
{

class GraspGeneratorTest
{
private:
  // A shared node handle
  ros::NodeHandle nh_;

  // Grasp generator
  hb_grasp_generator::CylindricalGraspGeneratorPtr simple_grasps_;

  // Tool for visualizing things in Rviz
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

  // Grasp filter
  hb_grasp_generator::GraspFilterPtr grasp_filter_;

  // data for generating grasps
  hb_grasp_generator::CylindricalGraspGeneratorOptions grasp_data_;

  // Shared planning scene (load once for everything)
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

  // which baxter arm are we using
  std::string arm_;
  std::string ee_group_name_;
  std::string planning_group_name_;

public:

  // Constructor
  GraspGeneratorTest()
    : nh_("~")
  {
    // Get arm info from param server
    nh_.param("arm", arm_, std::string("l_arm"));
    nh_.param("ee_group_name", ee_group_name_, std::string("l_gripper"));
    planning_group_name_ = "l_arm";

    ROS_INFO_STREAM_NAMED("test","Arm: " << arm_);
    ROS_INFO_STREAM_NAMED("test","End Effector: " << ee_group_name_);
    ROS_INFO_STREAM_NAMED("test","Planning Group: " << planning_group_name_);


    // ---------------------------------------------------------------------------------------------
    // Load planning scene to share
    planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));

    // ---------------------------------------------------------------------------------------------
    // Load the Robot Viz Tools for publishing to Rviz
    visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools("bender/base_link",
                                                                   "/grasp_test",
                                                                   planning_scene_monitor_));
    visual_tools_->setLifetime(40.0);
    const robot_model::JointModelGroup* ee_jmg = planning_scene_monitor_->getRobotModel()->getJointModelGroup("l_gripper");
    visual_tools_->loadEEMarker(ee_jmg);

    // ---------------------------------------------------------------------------------------------
    // Load grasp generator
    simple_grasps_.reset(new hb_grasp_generator::CylindricalGraspGenerator());

    // ---------------------------------------------------------------------------------------------
    // Load grasp filter
    const robot_state::RobotState& robot_state = planning_scene_monitor_->getPlanningScene()->getCurrentState();
    grasp_filter_.reset(new hb_grasp_generator::GraspFilter(robot_state, visual_tools_) );

    // ---------------------------------------------------------------------------------------------
    // Generate grasps for a bunch of random objects
    geometry_msgs::Pose object_pose;
    object_pose.position.x = 0.8;
    object_pose.position.y = -0.3;
    object_pose.position.z = 0.8;

    object_pose.orientation.x = 0.0;
    object_pose.orientation.y = 0.0;
    object_pose.orientation.z = 0.0;
    object_pose.orientation.w = 1.0;

    std::vector<moveit_msgs::Grasp> possible_grasps;
    std::vector<trajectory_msgs::JointTrajectoryPoint> ik_solutions; // save each grasps ik solution for visualization

    ROS_INFO_STREAM_NAMED("test","Adding object");

    possible_grasps.clear();
    ik_solutions.clear();

    // Generate set of grasps for one object
    simple_grasps_->generateGrasp( object_pose, possible_grasps);

    // Filter the grasp for only the ones that are reachable
    while (ros::ok())
    {
      // Show the block
      visual_tools_->publishCylinder(object_pose, rviz_visual_tools::BLUE, 0.05, 0.05);

      bool filter_pregrasps = true;
      grasp_filter_->filterGrasps(possible_grasps, ik_solutions, filter_pregrasps, "bender/l_wrist_pitch_link", planning_group_name_);

      // Visualize them
      const robot_model::JointModelGroup* ee_jmg = planning_scene_monitor_->getRobotModel()->getJointModelGroup("l_gripper");
      visual_tools_->publishAnimatedGrasps(possible_grasps, ee_jmg);
      const robot_model::JointModelGroup* arm_jmg = planning_scene_monitor_->getRobotModel()->getJointModelGroup(planning_group_name_);
      visual_tools_->publishIKSolutions(ik_solutions, arm_jmg, 0.25);
      ros::Duration(0.5).sleep();
    }
  }

}; // end of class

} // namespace


int main(int argc, char *argv[])
{
  int num_tests = 5;

  ros::init(argc, argv, "grasp_filter_test");

  // Allow the action server to recieve and send ros messages
  ros::AsyncSpinner spinner(2);
  spinner.start();
  
  // Seed random
  srand(ros::Time::now().toSec());

  // Benchmark time
  ros::Time start_time;
  start_time = ros::Time::now();

  // Run Tests
  hb_grasp_generator::GraspGeneratorTest tester;

  // Benchmark time
  double duration = (ros::Time::now() - start_time).toNSec() * 1e-6;
  ROS_INFO_STREAM_NAMED("","Total time: " << duration);

  ros::Duration(1.0).sleep(); // let rviz markers finish publishing

  return 0;
}
