#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"

#include <string>
#include <vector>
#include <iostream>
#include "gravitational_torque_estimation.h"


std::vector<std::string> joint_names;
std::vector<double> joint_pos;
std::vector<double> joint_torque;
std::vector<double> current_torque;
trajectory_evaluation::GravitationalTorqueEstimationPtr torque_estimation;

void jointStatesCb(const sensor_msgs::JointState::ConstPtr& msg)
{
  std::size_t joint_idx = 0;
  for (std::vector<std::string>::iterator joint_name = joint_names.begin(); joint_name != joint_names.end(); ++joint_name)
  {

    // Find chain element on the message
    std::size_t msg_idx = std::find(msg->name.begin(), msg->name.end(), *joint_name) - msg->name.begin();
    if (msg_idx >= msg->name.size())
    {
      // Drop message
      ROS_WARN_STREAM("JointState message doesnt contain: " << *joint_name);
      return;
    }
    // Update joint position
    joint_pos[joint_idx] = msg->position[msg_idx];
    // Update torque estimation
    current_torque[joint_idx] = msg->effort[msg_idx];
    joint_idx += 1;
  }
  // Torque estimation
  torque_estimation->estimate(joint_pos, joint_torque);
  for (std::size_t i = 0; i < joint_names.size(); ++i)
  {
    std::cout << joint_pos[i] << "," << joint_torque[i] << "," << current_torque[i] << std::endl;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "online_torque_estimation");
  ros::NodeHandle nh;
  ros::NodeHandle nhpriv("~");

  urdf::Model model;
  if (!model.initParam("robot_description"))
  {
    ROS_ERROR("Failed to parse URDF");
    return -1;
  }

  /* Link parameters */
  std::string root_link = "base_link";
  std::string tip_link = "tip_link";
  nhpriv.param<std::string>("root_link", root_link, "base_link");
  nhpriv.param<std::string>("tip_link", tip_link, "tip_link");
  ROS_INFO_STREAM("Root link: " << root_link);
  ROS_INFO_STREAM("Tip link: " << tip_link);  
  torque_estimation.reset(new trajectory_evaluation::GravitationalTorqueEstimation(model, root_link, tip_link)); 
  unsigned int dof = torque_estimation->getDOF();
  ROS_INFO_STREAM("DOF: " << dof);
  joint_pos.resize(dof, 0.0);
  joint_torque.resize(dof, 0.0);
  current_torque.resize(dof, 0.0);
  // Print chain names
  joint_names = torque_estimation->getJointNames();

  ROS_INFO("Joint names: ");
  for (std::vector<std::string>::iterator i = joint_names.begin(); i != joint_names.end(); ++i)
  {
    ROS_INFO_STREAM("\t" << *i << " max effort: " << model.getJoint(*i)->limits->effort);

  }
  ros::Subscriber sub = nh.subscribe("/bender/joint_states", 10, jointStatesCb);
  ros::spin();


  return 0;
}
