#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "control_msgs/FollowJointTrajectoryActionGoal.h"
#include "control_msgs/FollowJointTrajectoryGoal.h"

#include <string>
#include <vector>
#include "gravitational_torque_estimation.h"

#include <iostream>
#include <fstream>


std::vector<std::string> joint_names;
std::vector<double> joint_pos;
std::vector<double> penalty_ponderation;
std::vector<double> position_max_limits;
std::vector<double> position_min_limits;
std::vector<double> range;
trajectory_evaluation::GravitationalTorqueEstimationPtr torque_estimation; 

int i;
double position_penalty_index = 1;



//std::string datos = "";

void score(const control_msgs::FollowJointTrajectoryActionGoal::ConstPtr& msg, int index_point )
{
  

  std::size_t joint_idx = 0;
  for (std::vector<std::string>::iterator joint_name = joint_names.begin(); joint_name != joint_names.end(); ++joint_name)
  {

    // Find chain element on the message
    std::size_t msg_idx = std::find(msg->goal.trajectory.joint_names.begin(), msg->goal.trajectory.joint_names.end(), *joint_name) - msg->goal.trajectory.joint_names.begin();
    if (msg_idx >= msg->goal.trajectory.joint_names.size())
    {
      // Drop message
      ROS_WARN_STREAM("JointState message doesnt contain: " << *joint_name);
      return;
    }

    joint_pos[joint_idx++] = msg->goal.trajectory.points[index_point].positions[msg_idx];

  }



  // Torque estimation & penalty
  double penalty_multiplier_ = 1.0;
  double joint_limits_multiplier(1.0);
  double lower_bound_distance;
  double upper_bound_distance;
  range.resize(6);

  for (std::size_t i = 0; i < joint_names.size(); ++i)
  {
    // Joint saturation
    double joint_sat = std::min(position_max_limits[i], std::max(joint_pos[i], position_min_limits[i]));
    ROS_INFO_STREAM("" << joint_names[i] << "\t " << joint_sat);
    upper_bound_distance = position_max_limits[i] - joint_sat;
    lower_bound_distance = joint_sat - position_min_limits[i];

    //Saturación
    // if(upper_bound_distance<0){upper_bound_distance = 0;}
    // if(lower_bound_distance<0){ lower_bound_distance = 0;}


    range[i] = position_max_limits[i] - position_min_limits[i];
    
    joint_limits_multiplier *= (lower_bound_distance * upper_bound_distance / (range[i] * range[i]));
  
  }
    position_penalty_index = 1;
    position_penalty_index =  (1.0 - exp(-penalty_multiplier_ * joint_limits_multiplier));
    ROS_INFO_STREAM("-----------------------------------------");
    //ROS_INFO_STREAM("torque penalty index: " << torque_penalty_index);
    ROS_INFO_STREAM("position score index: " << (position_penalty_index*1000000000)/2439.67);
    ROS_INFO_STREAM("-----------------------------------------");
}



void jointStatesCb(const control_msgs::FollowJointTrajectoryActionGoal::ConstPtr& msg)
{

  ROS_INFO_STREAM("Entro a callback y se iniciara score");

  int trajectory_size=msg->goal.trajectory.points.size();

  ROS_INFO_STREAM("Cantidad de via points: " << trajectory_size);


  double trajectory_score=0;
  
  for (i=0;i<trajectory_size;i++)
  {

  score(msg,i);


  double point_score;
  point_score=(position_penalty_index*1000000000)/2439.67; //13.33

  if (point_score>100)
  {
    point_score=100;
  }

  trajectory_score=trajectory_score+point_score;

  if (position_penalty_index==0)
  {
    trajectory_score=0;
    break; 
  }

  }
  ROS_INFO_STREAM("trajectory score: " << trajectory_score/trajectory_size);
  ROS_INFO_STREAM("-----------------------------------------");
    
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
  penalty_ponderation.resize(dof, 0.0);
  position_max_limits.resize(dof, 0.0);
  position_min_limits.resize(dof, 0.0);

  double security_factor =1;

  // Print chain names
  joint_names = torque_estimation->getJointNames();
  ROS_INFO("Joint names: ");
  int j=0;
  for (std::vector<std::string>::iterator i = joint_names.begin(); i != joint_names.end(); ++i)
  {
    ROS_INFO_STREAM("\t" << *i);
    position_max_limits[j] = (model.getJoint(*i)->limits->upper)*security_factor;
    position_min_limits[j++] = (model.getJoint(*i)->limits->lower)*security_factor;
    
  }
  ros::Subscriber sub = nh.subscribe("/bender/l_arm_controller/follow_joint_trajectory/goal", 10, jointStatesCb);
  ros::spin();


  return 0;
}