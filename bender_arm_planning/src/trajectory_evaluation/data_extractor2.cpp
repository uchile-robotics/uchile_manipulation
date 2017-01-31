
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
std::string datos = "";

void joint_state_cb(const sensor_msgs::JointStatePtr &msg)
{
    
     joint_state->position = msg->position;
   
}



int main(int argc, char **argv)
{
  ros::init (argc, argv, "JointState_filter");
  ros::NodeHandle nh;

  joint_state.reset(new sensor_msgs::JointState);
  joint_state->position.resize(6);

  
  

  ros::Subscriber joint_subs = nh.subscribe("/bender/l_arm/joint_states", 1, joint_state_cb);

  ros::Publisher chatter_pub = nh.advertise<sensor_msgs::JointState>("bender/joint_state_filter", 1000);
  

  ros::AsyncSpinner spinner(1);
  spinner.start();


  ros::Rate rate(30);
  while(ros::ok())
  {
    

    
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
