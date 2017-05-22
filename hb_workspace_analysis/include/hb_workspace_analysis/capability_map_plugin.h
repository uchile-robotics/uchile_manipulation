/* 
* Get grasp positions from MongoDB
*
* Author: Rodrigo Munoz
*/

// Header guard
#ifndef CAPABILITY_MAP_PLUGIN
#define CAPABILITY_MAP_PLUGIN

// Interface with MoveGroup
#include <moveit/move_group/move_group_capability.h>
#include <bender_arm_planning/CapabilityMapGrasp.h>

namespace move_group
{

// Service name
static const std::string CAPABILITY_MAP_PLUGIN_NAME = "capability_map";

class CapabilityMapPlugin : public MoveGroupCapability
{
public:

  CapabilityMapPlugin();

  virtual void initialize();

private:

  bool getGraspsCb(bender_arm_planning::CapabilityMapGrasp::Request &req, bender_arm_planning::CapabilityMapGrasp::Response &res);
  bool filterPregraspCollision(std::vector<trajectory_msgs::JointTrajectoryPoint>& pregrasps, const std::string& group_name);
  // Servicio
  ros::ServiceServer grasp_service_;
  // Capability map reference frame
  std::string ref_frame_;
};

}

// CAPABILITY_MAP_PLUGIN
#endif 
