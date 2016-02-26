/* 
* Permite obtener grasps precalculados
*
* Autor: Rodrigo Munoz
*/

// Header guard
#ifndef BENDER_ARM_PLANNING_CAPMAP_SERVICE_
#define BENDER_ARM_PLANNING_CAPMAP_SERVICE_

// Mensajes y servicio
#include <bender_arm_planning/CapabilityMapGrasp.h>
#include <bender_arm_planning/CapabilityMapBestDistance.h>
#include <bender_arm_planning/GraspPosition.h>
// Capability Map
#include <bender_arm_planning/capability_map.h>
#include <ros/ros.h>

namespace bender_arm_planning
{

static const std::string CAPABILITY_MAP_SERVICE_NAME = "capability_map"; // Nombre de servicio

class CapabilityMapService
{
public:
  CapabilityMapService(const ros::NodeHandle& nh);

private:
  ros::NodeHandle nh_;

  bool getGraspsCb(CapabilityMapGrasp::Request &req, CapabilityMapGrasp::Response &res);

  void addQuality(std::vector<GraspPosition>& grasps, const geometry_msgs::PoseStamped& target_pose) const;
  bool getTargetPose(const moveit_msgs::CollisionObject& target, geometry_msgs::PoseStamped& target_pose, double& target_height) const;

  bool getBestDistanceCb(CapabilityMapBestDistance::Request &req, CapabilityMapBestDistance::Response &res);
  // Capability Map
  capability_map::CapabilityMapPtr capability_map_;
  // Servicio
  ros::ServiceServer grasp_service_;
  ros::ServiceServer distance_service_;
  // Resolucion del Cap Map
  double z_res_, x_res_;
  // Frame de referencia del CM
  std::string ref_frame_;
};

}

// BENDER_ARM_PLANNING_CAPMAP_SERVICE_
#endif 
