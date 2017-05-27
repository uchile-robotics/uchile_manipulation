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
#include <hb_workspace_analysis/GraspStorage.h>
#include <mongo_ros/message_collection.h>
#include <hb_workspace_analysis/GetCapabilityMap.h>

namespace move_group
{

// Service name
static const std::string CAPABILITY_MAP_PLUGIN_NAME = "capability_map";
// Typedef for DB
typedef mongo_ros::MessageCollection<hb_workspace_analysis::GraspStorage> GraspStorageDb;
typedef mongo_ros::MessageWithMetadata<hb_workspace_analysis::GraspStorage> GraspStorageWithMetadata;
typedef boost::shared_ptr<const GraspStorageWithMetadata> GraspStorageWithMetadataPtr;


  class CapabilityMapPlugin : public MoveGroupCapability
{
public:

  CapabilityMapPlugin();

  virtual void initialize();

private:

  bool getCapabilityMapCb(hb_workspace_analysis::GetCapabilityMap::Request &req,
                          hb_workspace_analysis::GetCapabilityMap::Response &res);
  bool filterPregraspCollision(std::vector<trajectory_msgs::JointTrajectoryPoint>& pregrasps, const std::string& group_name);
  // Servicio
  ros::ServiceServer grasp_service_;
  // Capability map reference frame
  std::string ref_frame_;
  // DB connection
  boost::shared_ptr<GraspStorageDb> db_;
  // Capability map resolution
  double resolution_;
  // Search factor
  double search_factor_;

};

}

// CAPABILITY_MAP_PLUGIN
#endif 
