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
#include <mongo_ros/message_collection.h>
// Capability map
#include <hb_workspace_analysis/GetCapabilityMap.h>
#include <hb_workspace_analysis/GraspStorage.h>
#include <hb_workspace_analysis/capability_map_options.h>
#include <hb_grasp_generator/grasp_generator.h>
#include <hb_grasp_generator/grasp_filter.h>

namespace move_group
{

// Service name
static const std::string CAPABILITY_MAP_PLUGIN_NAME = "capability_map";
// Typedef for DB
typedef mongo_ros::MessageCollection<hb_workspace_analysis::GraspStorage> GraspStorageDb;
typedef boost::shared_ptr<GraspStorageDb> GraspStorageDbPtr;
typedef mongo_ros::MessageWithMetadata<hb_workspace_analysis::GraspStorage> GraspStorageWithMetadata;
typedef boost::shared_ptr<const GraspStorageWithMetadata> GraspStorageWithMetadataPtr;
typedef std::map<std::string, GraspStorageDbPtr> DatabaseTable;
typedef std::map<std::string, hb_workspace_analysis::CapabilityMapOptions> CapabilityMapOptionsTable;
typedef std::map<std::string, hb_grasp_generator::CylindricalGraspGeneratorPtr> GraspGeneratorTable;
typedef std::map<std::string, hb_grasp_generator::GraspFilterPtr> GraspFilterTable;

class CapabilityMapPlugin : public MoveGroupCapability
{
public:

  CapabilityMapPlugin();

  virtual void initialize();

private:

  bool getCapabilityMapCb(hb_workspace_analysis::GetCapabilityMap::Request &req,
                          hb_workspace_analysis::GetCapabilityMap::Response &res);

  bool filterPregraspCollision(const std::string& group_name,
                               const std::vector<std::string>& name, const std::vector<double>& position,
                               robot_state::RobotState& robot_state);

  bool checkSelfCollision(planning_scene_monitor::LockedPlanningSceneRO& ls, const std::string& group_name,
                          const std::vector<std::string>& name, const std::vector<double>& position,
                          robot_state::RobotState& robot_state);

  bool checkCollision(planning_scene_monitor::LockedPlanningSceneRO& ls, const std::string& group_name,
                      const std::vector<std::string>& name, const std::vector<double>& position,
                      robot_state::RobotState& robot_state);

  // Capability map options
  CapabilityMapOptionsTable capmap_opt_;
  // Service
  ros::ServiceServer grasp_service_;
  // Capability map reference frame
  std::string ref_frame_;
  // DB connections
  DatabaseTable db_;
  // Grasp generators
  GraspGeneratorTable grasp_gen_;
  // Grasp filter
  GraspFilterTable grasp_filter_;
  // TF transformer from context
  boost::shared_ptr<tf::Transformer> tf_;
  // Robot URDF
  boost::shared_ptr<urdf::ModelInterface> urdf_;

};

}

// CAPABILITY_MAP_PLUGIN
#endif 
