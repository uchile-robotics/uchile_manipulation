// ROS
#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <eigen_conversions/eigen_msg.h>
#include "hb_grasp_generator/grasp_generator.h"

namespace hb_grasp
{

class GraspGeneratorTest
{
private:
  // A shared node handle
  ros::NodeHandle nh_;

  // Grasp generator
  hb_grasp_generator::CylindricalGraspGeneratorPtr simple_grasps_;

  // class for publishing stuff to rviz
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

  // which baxter arm are we using
  std::string arm_;
  std::string ee_group_name_;
  std::string planning_group_name_;

public:

  // Constructor
  GraspGeneratorTest()
    : nh_("~")
  {
    nh_.param("arm", arm_, std::string("l_arm"));
    nh_.param("ee_group_name", ee_group_name_, std::string("l_gripper"));
    planning_group_name_ = "l_arm";

    ROS_INFO_STREAM_NAMED("test","Arm: " << arm_);
    ROS_INFO_STREAM_NAMED("test","End Effector: " << ee_group_name_);
    ROS_INFO_STREAM_NAMED("test","Planning Group: " << planning_group_name_);


    // ---------------------------------------------------------------------------------------------
    // Load the Robot Viz Tools for publishing to Rviz
    visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools("bender/base_link"));
    visual_tools_->setLifetime(120.0);

    visual_tools_->loadMarkerPub();

    // ---------------------------------------------------------------------------------------------
    // Load grasp generator
    simple_grasps_.reset( new hb_grasp_generator::CylindricalGraspGenerator() );

    geometry_msgs::Pose pose;
    visual_tools_->generateEmptyPose(pose);

    // ---------------------------------------------------------------------------------------------
    // Animate open and closing end effector

    for (std::size_t i = 0; i < 4; ++i)
    {
      // Test visualization of end effector in OPEN position
      //grasp_data_.setRobotStatePreGrasp( visual_tools_->getSharedRobotState() );


      const robot_model::JointModelGroup* ee_jmg = visual_tools_->getRobotModel()->getJointModelGroup("l_gripper");
      visual_tools_->loadEEMarker(ee_jmg);
      visual_tools_->publishEEMarkers(pose, ee_jmg, rviz_visual_tools::ORANGE, "test_eef");
      ros::Duration(1.0).sleep();

      // Test visualization of end effector in CLOSED position
      //grasp_data_.setRobotStateGrasp( visual_tools_->getSharedRobotState() );
      //const robot_model::JointModelGroup* ee_jmg = visual_tools_->getRobotModel()->getJointModelGroup("l_gripper");
      //visual_tools_->loadEEMarker(ee_jmg);
      visual_tools_->publishEEMarkers(pose, ee_jmg, rviz_visual_tools::GREEN, "test_eef");
      ros::Duration(1.0).sleep();      
    }

    // ---------------------------------------------------------------------------------------------
    // Generate grasps for a bunch of random objects
    geometry_msgs::Pose object_pose;
    std::vector<moveit_msgs::Grasp> possible_grasps;

    // Show the block
    visual_tools_->publishBlock(object_pose, rviz_visual_tools::BLUE, 0.1);

    possible_grasps.clear();

    // Generate set of grasps for one object
    simple_grasps_->generateGrasp(object_pose, possible_grasps);

    // Visualize them
    const robot_model::JointModelGroup* ee_jmg = visual_tools_->getRobotModel()->getJointModelGroup("l_gripper");
    visual_tools_->publishAnimatedGrasps(possible_grasps, ee_jmg);
    //visual_tools_->publishGrasps(possible_grasps, grasp_data_.ee_parent_link_);

  }
}; // end of class

} // namespace


int main(int argc, char *argv[])
{
  int num_tests = 10;
  ros::init(argc, argv, "grasp_generator_test");

  ROS_INFO_STREAM_NAMED("main","Simple Grasps Test");

  ros::AsyncSpinner spinner(2);
  spinner.start();

  // Seed random
  srand(ros::Time::now().toSec());

  // Benchmark time
  ros::Time start_time;
  start_time = ros::Time::now();

  // Run Tests
  hb_grasp::GraspGeneratorTest tester();

  // Benchmark time
  double duration = (ros::Time::now() - start_time).toNSec() * 1e-6;
  ROS_INFO_STREAM_NAMED("","Total time: " << duration);
  //std::cout << duration << "\t" << num_tests << std::endl;

  ros::Duration(1.0).sleep(); // let rviz markers finish publishing

  return 0;
}


/*
int main(int argc, char** argv)
{
  ros::init(argc, argv, "cylindrical_grasp_generator_test");
  ROS_INFO_STREAM("Grasp generator Demo");

  // Allow the action server to recieve and send ros messages
  ros::AsyncSpinner spinner(1);
  spinner.start();

  hb_grasp_generator::CylindricalGraspGenerator demo;

  // Create object pose
  geometry_msgs::Pose pose_msg;
  Eigen::Affine3d pose = Eigen::Affine3d::Identity() * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX())
                         * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY())
                         * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ());
  pose.translation().x() = 1.0; // Move one meter on x axis
  tf::poseEigenToMsg(pose, pose_msg);
  // Generate grasp positions
  std::vector<moveit_msgs::Grasp> grasps;
  ros::Time t0 = ros::Time::now();
  demo.generateGrasp(pose_msg, grasps);
  ROS_INFO_STREAM("Generated grasps: " << grasps.size() << " sec: " << (ros::Time::now() - t0).toSec());
  return 0;
}
*/
