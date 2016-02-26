/* 
* Permite limpiar, parar e iniciar Octomap usado para representar
* objetos de colision en PlanningScene.
* 
* Antes de emplear planificadores es util limpiar y parar la actualizacion
* de Octomap para evitar errores del planificador que hacen referencia
* a cambios en la escena durante la planificacion.
*
* Autor: Rodrigo Munoz
*/

#include <bender_arm_planning/manage_octomap_service_capability.h>
#include <bender_arm_planning/OctomapOptions.h>

namespace move_group
{

  ManageOctomapService::ManageOctomapService():
    MoveGroupCapability("ManageOctomapService")
  {
  }

  void ManageOctomapService::initialize()
  {
    service_ = root_node_handle_.advertiseService(MANAGE_OCTOMAP_SERVICE_NAME, &ManageOctomapService::manageOctomap, this);
    
    // Deshabiliar octomap al comienzo
    ROS_INFO("Stop octomap...");
    using namespace planning_scene_monitor;
    context_->planning_scene_monitor_->startWorldGeometryMonitor(PlanningSceneMonitor::DEFAULT_COLLISION_OBJECT_TOPIC,
      PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_WORLD_TOPIC, false);
  }

  bool ManageOctomapService::manageOctomap(bender_arm_planning::ManageOctomap::Request &req, bender_arm_planning::ManageOctomap::Response &res)
  {
    if (!context_->planning_scene_monitor_)
    {
      ROS_ERROR("Cannot clear octomap since planning_scene_monitor_ does not exist.");
      return true;
    }

    switch (req.option.val)
    {
      case bender_arm_planning::OctomapOptions::CLEAR:
        ROS_INFO("Clearing octomap...");
        context_->planning_scene_monitor_->clearOctomap();
        ROS_INFO("Octomap cleared.");
        break;

      case bender_arm_planning::OctomapOptions::STOP:
        ROS_INFO("Stop octomap...");
        context_->planning_scene_monitor_->stopWorldGeometryMonitor();
        ROS_INFO("Octomap stoped.");
        break;

      case bender_arm_planning::OctomapOptions::START:
        ROS_INFO("Start octomap...");
        context_->planning_scene_monitor_->startWorldGeometryMonitor();
        break;

      case bender_arm_planning::OctomapOptions::UPDATE:
        ROS_INFO("Update octomap...");
        context_->planning_scene_monitor_->startWorldGeometryMonitor();
        ros::spinOnce();
        ros::Duration(3.0).sleep();
        ros::spinOnce();
        context_->planning_scene_monitor_->clearOctomap();
        ros::Duration(3.0).sleep();
        context_->planning_scene_monitor_->stopWorldGeometryMonitor();
        break;

      default:
        ROS_WARN("Unknown request...");
        break;
    }

    
    return true;
  }

}


#include <class_loader/class_loader.h>
CLASS_LOADER_REGISTER_CLASS(move_group::ManageOctomapService, move_group::MoveGroupCapability)
