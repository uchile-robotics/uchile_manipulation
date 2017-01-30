
#include <ros/ros.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <moveit/kinematics_metrics/kinematics_metrics.h>
#include <Eigen/Eigenvalues>
#include <boost/math/constants/constants.hpp>
#include <sensor_msgs/JointState.h>



sensor_msgs::JointStatePtr joint_state;
std::vector<std::string> joint_names;


void joint_state_cb(const sensor_msgs::JointStatePtr &msg)
{
	//joint_state->position = current_joint_state->position;

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

double penalty_multiplier_ = 1.0;
double getJointLimitsPenalty(const robot_state::RobotState& state,
                                                 const robot_model::JointModelGroup* joint_model_group)
 {
   if (fabs(penalty_multiplier_) <= boost::math::tools::epsilon<double>())
     return 1.0;
   double joint_limits_multiplier(1.0);
   const std::vector<const robot_model::JointModel*>& joint_model_vector = joint_model_group->getJointModels();
   for (std::size_t i = 0; i < joint_model_vector.size(); ++i)
   {
     if (joint_model_vector[i]->getType() == robot_model::JointModel::REVOLUTE)
     {
       const robot_model::RevoluteJointModel* revolute_model =
           static_cast<const robot_model::RevoluteJointModel*>(joint_model_vector[i]);
       if (revolute_model->isContinuous())
         continue;
     }
     if (joint_model_vector[i]->getType() == robot_model::JointModel::PLANAR)
     {
       const robot_model::JointModel::Bounds& bounds = joint_model_vector[i]->getVariableBounds();
       if (bounds[0].min_position_ == -std::numeric_limits<double>::max() ||
           bounds[0].max_position_ == std::numeric_limits<double>::max() ||
           bounds[1].min_position_ == -std::numeric_limits<double>::max() ||
           bounds[1].max_position_ == std::numeric_limits<double>::max() ||
           bounds[2].min_position_ == -boost::math::constants::pi<double>() ||
           bounds[2].max_position_ == boost::math::constants::pi<double>())
         continue;
     }
     if (joint_model_vector[i]->getType() == robot_model::JointModel::FLOATING)
     {
       // Joint limits are not well-defined for floating joints
       continue;
     }
     const double* joint_values = state.getJointPositions(joint_model_vector[i]);
     const robot_model::JointModel::Bounds& bounds = joint_model_vector[i]->getVariableBounds();
     std::vector<double> lower_bounds, upper_bounds;
     for (std::size_t j = 0; j < bounds.size(); ++j)
     {
       lower_bounds.push_back(bounds[j].min_position_);
       upper_bounds.push_back(bounds[j].max_position_);
       //ROS_DEBUG_STREAM("limite superior "<< j <<upper_bounds[j]);
       //ROS_DEBUG_STREAM("limite inferior "<< j <<lower_bounds[j]);
     }
     double lower_bound_distance = joint_model_vector[i]->distance(joint_values, &lower_bounds[0]);
     double upper_bound_distance = joint_model_vector[i]->distance(joint_values, &upper_bounds[0]);
     double range = lower_bound_distance + upper_bound_distance;

     if (range <= boost::math::tools::epsilon<double>())
       continue;
     joint_limits_multiplier *= (lower_bound_distance * upper_bound_distance / (range * range));
   }
   return (1.0 - exp(-penalty_multiplier_ * joint_limits_multiplier));
 }

int main(int argc, char **argv)
{

  ros::init (argc, argv, "left_arm_kinematics");
  ros::NodeHandle nh;

  joint_state.reset(new sensor_msgs::JointState);
  joint_state->position.resize(6);

  

  
  // .. _RobotModelLoader: http://docs.ros.org/api/moveit_ros_planning/html/classrobot__model__loader_1_1RobotModelLoader.html
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  ROS_DEBUG("Model frame: %s", kinematic_model->getModelFrame().c_str());

  // Using the :moveit_core:`RobotModel`, we can construct a
  // :moveit_core:`RobotState` that maintains the configuration
  // of the robot. We will set all joints in the state to their
  // default values. We can then get a
  // :moveit_core:`JointModelGroup`, which represents the robot
  // model for a particular group, e.g. the "right_arm" of the PR2
  // robot.
  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
  kinematic_state->setToDefaultValues();
  const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("l_arm");


  ros::Subscriber joint_subs = nh.subscribe("/bender/joint_states", 1, joint_state_cb);

  ros::AsyncSpinner spinner(1);
  spinner.start();


  ros::Rate rate(30);
  while(ros::ok())
  {
		joint_names =	joint_model_group->getJointModelNames();

		std::vector<double> joint_values;
		kinematic_state->copyJointGroupPositions(joint_model_group,
				joint_values);
		for (std::size_t i = 0; i < joint_names.size(); ++i) {
			ROS_DEBUG("%s: %f", joint_names[i].c_str(), joint_state->position[i]);
		}

    ROS_DEBUG_STREAM("---------------------------");

		// Joint Limits
		// ^^^^^^^^^^^^
		// setJointGroupPositions() does not enforce joint limits by itself, but a call to enforceBounds() will do it.
		/* Set one joint in the right arm outside its joint limit */

    joint_values = joint_state->position; 
  

  	kinematic_state->setJointGroupPositions(joint_model_group, joint_values);
  //
  //		/* Check whether any joint is outside its joint limits */
  //		ROS_DEBUG_STREAM(
  //				"Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));
  //
  //		/* Enforce the joint limits for this state and check again*/
  //		kinematic_state->enforceBounds();
  //		ROS_DEBUG_STREAM(
  //				"Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid")); 

  	kinematics_metrics::KinematicsMetricsPtr kin_metrics(
  			new kinematics_metrics::KinematicsMetrics(kinematic_model));  

 		double penalty = getJointLimitsPenalty(*kinematic_state, joint_model_group);
  	double penalty_norm= (penalty*100000);             // Esta normalizado para el kuka ... revisar limites de bender ...
  	ROS_DEBUG_STREAM("Penalty: " << penalty_norm); 

  	double score = (penalty*1000000000)/2439.67; //20.8 // con todos los joint en su centro... 11.085
  	ROS_INFO_STREAM("score: " << score);
    //std::cout << (penalty*10000000) << std::endl;
  		
  	ROS_DEBUG_STREAM("---------------------------"); 
    
    double score_limit=20;
    if (score>score_limit)
    {
        ROS_DEBUG_STREAM("PASS");
    }
    else{
        ROS_DEBUG_STREAM("FAIL");
    }
    ROS_DEBUG_STREAM("---------------------------"); 

    rate.sleep();

  }
  return 0;
}
