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
    nh_.param("arm", planning_group_name_, std::string("l_arm"));
    nh_.param("ee_name", ee_group_name_, std::string("l_gripper"));

    ROS_INFO_STREAM_NAMED("test","Planning Group: " << planning_group_name_);
    ROS_INFO_STREAM_NAMED("test","End Effector: " << ee_group_name_);


    // ---------------------------------------------------------------------------------------------
    // Load planning scene to share
    planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));

    // ---------------------------------------------------------------------------------------------
    // Load the Robot Viz Tools for publishing to Rviz
    visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools("bender/base_link",
                                                                   "/grasp_test",
                                                                   planning_scene_monitor_));

    visual_tools_->setLifetime(40.0);
    // Get joint model groups
    const robot_model::JointModelGroup
        *ee_jmg = planning_scene_monitor_->getRobotModel()->getJointModelGroup(ee_group_name_);
    const robot_model::JointModelGroup
        *arm_jmg = planning_scene_monitor_->getRobotModel()->getJointModelGroup(planning_group_name_);
    visual_tools_->loadEEMarker(ee_jmg);

    // ---------------------------------------------------------------------------------------------
    // Load grasp options
    hb_grasp_generator::GraspOptions opt;
    opt.load(nh_, ee_group_name_);
    ros::NodeHandle grasp_nh(nh_, ee_group_name_);
    simple_grasps_.reset(new hb_grasp_generator::CylindricalGraspGenerator(grasp_nh, opt));


    // ---------------------------------------------------------------------------------------------
    // Load Grasp filter
    const robot_state::RobotState& robot_state = planning_scene_monitor_->getPlanningScene()->getCurrentState();
    grasp_filter_.reset(new hb_grasp_generator::GraspFilter(robot_state, visual_tools_) );

    // ---------------------------------------------------------------------------------------------
    // Test grasp position
    geometry_msgs::Pose object_pose;
    object_pose.position.x = 0.50;
    object_pose.position.y = 0.29;
    object_pose.position.z = 0.60;

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
    simple_grasps_->generateGrasp(object_pose, possible_grasps);

    // Publish cylinder
    visual_tools_->setLifetime(120.0);
    visual_tools_->enableBatchPublishing(true);
    visual_tools_->publishCylinder(object_pose, rviz_visual_tools::BLUE, 0.05, 0.05);
    visual_tools_->triggerBatchPublish();

    // Apply grasp filter
    bool filter_pregrasps = true;
    double ik_timeout = 0.1;
    grasp_filter_->filterGrasps(possible_grasps,
                                ik_solutions,
                                filter_pregrasps,
                                opt.end_effector_parent_link,
                                planning_group_name_, ik_timeout);

    // Visualize IK solutions
    visual_tools_->publishIKSolutions(ik_solutions, arm_jmg, 0.25);
  }
}; // end of class

} // namespace

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "grasp_filter_test");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Wait for RViz
  ros::Duration(5.0).sleep();
  // Benchmark time
  ros::Time start_time = ros::Time::now();

  // Run Tests
  hb_grasp_generator::GraspGeneratorTest tester;

  // Benchmark time
  ROS_INFO_STREAM("Total time: " << (ros::Time::now() - start_time).toSec());

  return 0;
}
