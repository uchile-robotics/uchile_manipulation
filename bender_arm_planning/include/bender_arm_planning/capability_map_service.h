/* 
* Permite obtener grasps precalculados
*
* Autor: Rodrigo Munoz
*/

// Header guard
#ifndef BENDER_PLANNING_CAPABILITY_MAP_SERVICE_
#define BENDER_PLANNING_CAPABILITY_MAP_SERVICE_

// Interface MoveGroup
#include <moveit/move_group/move_group_capability.h>
// Mensajes y servicio
#include <bender_arm_planning/CapabilityMapGrasp.h>
#include <bender_arm_planning/CapabilityMapBestDistance.h>
#include <bender_arm_planning/GraspPosition.h>
#include <bender_arm_planning/AddObjectACM.h>
// Capability Map
#include <bender_arm_planning/capability_map.h>

namespace move_group
{

static const std::string CAPABILITY_MAP_SERVICE_NAME = "capability_map"; // Nombre de servicio

class CapabilityMapService : public MoveGroupCapability
{
public:

  CapabilityMapService();

  virtual void initialize();

private:
  bool addObjectACMCb(bender_arm_planning::AddObjectACM::Request &req, bender_arm_planning::AddObjectACM::Response &res);
  bool getGraspsCb(bender_arm_planning::CapabilityMapGrasp::Request &req, bender_arm_planning::CapabilityMapGrasp::Response &res);

  void addQuality(std::vector<bender_arm_planning::GraspPosition>& grasps, const geometry_msgs::PoseStamped& target_pose) const;
  bool getTargetPose(const moveit_msgs::CollisionObject& target, geometry_msgs::PoseStamped& target_pose, double& target_height) const;

  bool getBestDistanceCb(bender_arm_planning::CapabilityMapBestDistance::Request &req, bender_arm_planning::CapabilityMapBestDistance::Response &res);

  bool filterPregraspCollision(std::vector<trajectory_msgs::JointTrajectoryPoint>& pregrasps, const std::string& group_name);
  // Capability Map
  capability_map::CapabilityMapPtr capability_map_;
  // Servicio
  ros::ServiceServer grasp_service_;
  ros::ServiceServer distance_service_;
  ros::ServiceServer acm_service_;
  // Resolucion del Cap Map
  double z_res_, x_res_;
  // Frame de referencia del CM
  std::string ref_frame_;
};

}

// BENDER_PLANNING_CAPABILITY_MAP_SERVICE_
#endif 
