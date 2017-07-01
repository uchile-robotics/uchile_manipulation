
#include <ros/ros.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <moveit/kinematics_metrics/kinematics_metrics.h>
#include <Eigen/Eigenvalues>
#include <boost/math/constants/constants.hpp>
#include <sensor_msgs/JointState.h>

#include "std_msgs/String.h"

#include <sstream>



sensor_msgs::JointStatePtr joint_state;
std::vector<std::string> joint_names;
double score2=0;

std::string datos = "";

void joint_state_cb(const sensor_msgs::JointStatePtr &msg)
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
     }
     joint_state->position[joint_idx++] = msg->position[msg_idx];
   }
}



int main(int argc, char **argv)
{
  ros::init (argc, argv, "JointState_filter");
  ros::NodeHandle nh;

  joint_state.reset(new sensor_msgs::JointState);
  joint_state->position.resize(6);

  // .. _RobotModelLoader: http://docs.ros.org/api/moveit_ros_planning/html/classrobot__model__loader_1_1RobotModelLoader.html
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  ROS_DEBUG("Model frame: %s", kinematic_model->getModelFrame().c_str());

  
  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
  kinematic_state->setToDefaultValues();
  const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("l_arm");


  ros::Subscriber joint_subs = nh.subscribe("/bender/joint_states", 1, joint_state_cb);

  ros::Publisher chatter_pub = nh.advertise<sensor_msgs::JointState>("bender/joint_state_filter", 1000);
  

  ros::AsyncSpinner spinner(1);
  spinner.start();


  ros::Rate rate(30);
  while(ros::ok())
  {
    joint_names = joint_model_group->getJointModelNames();

    std::vector<double> joint_values;
    kinematic_state->copyJointGroupPositions(joint_model_group,
        joint_values);
    for (std::size_t i = 0; i < joint_names.size(); ++i) {
      ROS_DEBUG("%s: %f", joint_names[i].c_str(), joint_state->position[i]);
    }

    joint_values = joint_state->position; 
  

    kinematic_state->setJointGroupPositions(joint_model_group, joint_values);
  
    kinematics_metrics::KinematicsMetricsPtr kin_metrics(
        new kinematics_metrics::KinematicsMetrics(kinematic_model));  
    

    // Extracción de datos
    ROS_INFO_STREAM("Desea guardar?");
    std::string enter="";
    std::cin>>enter;

    if(enter=="y")
    {
      
      chatter_pub.publish(joint_state);
    ROS_INFO_STREAM("JointState published!");
    // FIN EXTRACCION DE DATOS
      
    }
  

    rate.sleep();

  }
  return 0;
}
