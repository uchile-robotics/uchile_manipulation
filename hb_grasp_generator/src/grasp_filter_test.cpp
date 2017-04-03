// ROS
#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/JointState.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

// MoveIt
#include <moveit_msgs/MoveGroupAction.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/RobotState.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

// Rviz
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// Grasp
#include <moveit_visual_tools/moveit_visual_tools.h>

// Baxter specific properties
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
  hb_grasp_generator::GraspFilter grasp_filter_;

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
  GraspGeneratorTest(int num_tests) 
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
    visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools(grasp_data_.base_link_, "/grasp_test", planning_scene_monitor_));
    visual_tools_->setLifetime(40.0);
    visual_tools_->loadEEMarker("l_gripper");
    visual_tools_->setFloorToBaseHeight(-0.9);

    // Clear out old collision objects just because
    //visual_tools_->removeAllCollisionObjects();
// ---------------------------------------------------------------------------------------------
    // Load grasp generator
    simple_grasps_.reset( new hb_grasp_generator::CylindricalGraspGeneratorPtr(visual_tools_) );

    // ---------------------------------------------------------------------------------------------
    // Load grasp filter
    robot_state::RobotState robot_state = planning_scene_monitor_->getPlanningScene()->getCurrentState();
    grasp_filter_.reset(new moveit_simple_grasps::GraspFilter(robot_state, visual_tools_) );

    // ---------------------------------------------------------------------------------------------
    // Generate grasps for a bunch of random objects
    geometry_msgs::Pose object_pose;
    std::vector<moveit_msgs::Grasp> possible_grasps;
    std::vector<trajectory_msgs::JointTrajectoryPoint> ik_solutions; // save each grasps ik solution for visualization

    // Loop
    for (int i = 0; i < num_tests; ++i)
    {
      ROS_INFO_STREAM_NAMED("test","Adding random object " << i+1 << " of " << num_tests);

      // Remove randomness when we are only running one test
      if (num_tests == 1)
        generateTestObject(object_pose);
      else
        generateRandomObject(object_pose);

      // Show the block
      visual_tools_->publishBlock(object_pose, rviz_visual_tools::BLUE, BLOCK_SIZE);

      possible_grasps.clear();
      ik_solutions.clear();

      // Generate set of grasps for one object
      simple_grasps_->generateBlockGrasps( object_pose, grasp_data_, possible_grasps);

      // Filter the grasp for only the ones that are reachable
      bool filter_pregrasps = true;
      grasp_filter_->filterGrasps(possible_grasps, ik_solutions, filter_pregrasps, grasp_data_.ee_parent_link_, planning_group_name_);

      // Visualize them
      const robot_model::JointModelGroup* ee_jmg = planning_scene_monitor_->getRobotModel()->getJointModelGroup(grasp_data_.ee_group_);
      visual_tools_->publishAnimatedGrasps(possible_grasps, ee_jmg);
      const robot_model::JointModelGroup* arm_jmg = planning_scene_monitor_->getRobotModel()->getJointModelGroup(planning_group_name_);
      visual_tools_->publishIKSolutions(ik_solutions, arm_jmg, 0.25);


      // Make sure ros is still going
      if(!ros::ok())
        break;
    }


  }

  void generateTestObject(geometry_msgs::Pose& object_pose)
  {
    // Position
    geometry_msgs::Pose start_object_pose;
    geometry_msgs::Pose end_object_pose;

    start_object_pose.position.x = 0.8;
    start_object_pose.position.y = -0.5;
    start_object_pose.position.z = 0.02;

    end_object_pose.position.x = 0.25;
    end_object_pose.position.y = 0.15;
    end_object_pose.position.z = 0.02;

    // Orientation
    double angle = M_PI / 1.5;
    Eigen::Quaterniond quat(Eigen::AngleAxis<double>(double(angle), Eigen::Vector3d::UnitZ()));
    start_object_pose.orientation.x = quat.x();
    start_object_pose.orientation.y = quat.y();
    start_object_pose.orientation.z = quat.z();
    start_object_pose.orientation.w = quat.w();

    angle = M_PI / 1.1;
    quat = Eigen::Quaterniond(Eigen::AngleAxis<double>(double(angle), Eigen::Vector3d::UnitZ()));
    end_object_pose.orientation.x = quat.x();
    end_object_pose.orientation.y = quat.y();
    end_object_pose.orientation.z = quat.z();
    end_object_pose.orientation.w = quat.w();

    // Choose which object to test
    object_pose = start_object_pose;
  }

  void generateRandomObject(geometry_msgs::Pose& object_pose)
  {
    // Position
    object_pose.position.x = visual_tools_->dRand(0.7,TABLE_DEPTH);
    object_pose.position.y = visual_tools_->dRand(-TABLE_WIDTH/2,-0.1);
    object_pose.position.z = TABLE_Z + TABLE_HEIGHT / 2.0 + BLOCK_SIZE / 2.0;
  
    // Orientation
    double angle = M_PI * visual_tools_->dRand(0.1,1);
    Eigen::Quaterniond quat(Eigen::AngleAxis<double>(double(angle), Eigen::Vector3d::UnitZ()));
    object_pose.orientation.x = quat.x();
    object_pose.orientation.y = quat.y();
    object_pose.orientation.z = quat.z();
    object_pose.orientation.w = quat.w();
  }

}; // end of class

} // namespace


int main(int argc, char *argv[])
{
  int num_tests = 5;

  ros::init(argc, argv, "grasp_generator_test");

  // Allow the action server to recieve and send ros messages
  ros::AsyncSpinner spinner(2);
  spinner.start();

  // Check for verbose flag
  bool verbose = false;
  if (argc > 1)
  {
    for (std::size_t i = 0; i < argc; ++i)
    {
      if (strcmp(argv[i], "--verbose") == 0)
      {
        ROS_INFO_STREAM_NAMED("main","Running in VERBOSE mode (much slower)");
        verbose = true;
      }
    }
  }
  
  // Seed random
  srand(ros::Time::now().toSec());

  // Benchmark time
  ros::Time start_time;
  start_time = ros::Time::now();

  // Run Tests
  moveit_simple_grasps::GraspGeneratorTest tester(num_tests);

  // Benchmark time
  double duration = (ros::Time::now() - start_time).toNSec() * 1e-6;
  ROS_INFO_STREAM_NAMED("","Total time: " << duration);
  std::cout << duration << "\t" << num_tests << std::endl;

  ros::Duration(1.0).sleep(); // let rviz markers finish publishing

  return 0;
}
