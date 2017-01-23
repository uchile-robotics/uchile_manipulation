#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"

#include <string>
#include <vector>
#include "gravitational_torque_estimation.h"


std::vector<std::string> joint_names;
std::vector<double> joint_pos;
std::vector<double> joint_torque;
std::vector<double> torque_max_limits;
std::vector<double> torque_min_limits;
std::vector<double> range;
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
    joint_pos[joint_idx++] = msg->position[msg_idx];
  }
  // Torque estimation & penalty
  double penalty_multiplier_ = 1.0;
  double joint_limits_multiplier(1.0);
  double lower_bound_distance;
  double upper_bound_distance;

  torque_estimation->estimate(joint_pos, joint_torque);
  for (std::size_t i = 0; i < joint_names.size(); ++i)
  {
    ROS_INFO_STREAM("\t" << joint_names[i] << "\t" << joint_torque[i]);
    upper_bound_distance = torque_max_limits[i] - joint_torque[i];
    lower_bound_distance = joint_torque[i] - torque_min_limits[i];
    range[i] = torque_max_limits[i] - torque_min_limits[i];
    joint_limits_multiplier *= (lower_bound_distance * upper_bound_distance / (range[i] * range[i]));
  }
    double torque_penalty_index;
    torque_penalty_index =  (1.0 - exp(-penalty_multiplier_ * joint_limits_multiplier));
    ROS_INFO_STREAM("torque penalty index: " << torque_penalty_index);


}

int main(int argc, char **argv)
{

  // Torques limites
  torque_max_limits[0] = 10; torque_min_limits[0] = 10;
  torque_max_limits[1] = 10; torque_min_limits[1] = 10;
  torque_max_limits[2] = 10; torque_min_limits[2] = 10;
  torque_max_limits[3] = 10; torque_min_limits[3] = 10;
  torque_max_limits[4] = 10; torque_min_limits[4] = 10;
  torque_max_limits[5] = 10; torque_min_limits[5] = 10;
  torque_max_limits[6] = 10; torque_min_limits[6] = 10;
  torque_max_limits[7] = 10; torque_min_limits[7] = 10;
  torque_max_limits[8] = 10; torque_min_limits[8] = 10;
  torque_max_limits[9] = 10; torque_min_limits[9] = 10;


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
  // Print chain names
  joint_names = torque_estimation->getJointNames();
  ROS_INFO("Joint names: ");
  for (std::vector<std::string>::iterator i = joint_names.begin(); i != joint_names.end(); ++i)
  {
    ROS_INFO_STREAM("\t" << *i);
  }
  ros::Subscriber sub = nh.subscribe("/bender/joint_states", 10, jointStatesCb);
  ros::spin();


  return 0;
}