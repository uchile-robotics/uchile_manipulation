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

// Header guard
#ifndef BENDER_PLANNING_MANAGE_OCTOMAP_CAPABILITY_
#define BENDER_PLANNING_MANAGE_OCTOMAP_CAPABILITY_

#include <moveit/move_group/move_group_capability.h>
#include <bender_arm_planning/ManageOctomap.h>

namespace move_group
{

static const std::string MANAGE_OCTOMAP_SERVICE_NAME = "manage_octomap"; // Nombre de servicio

class ManageOctomapService : public MoveGroupCapability
{
public:

  ManageOctomapService();

  virtual void initialize();

private:

  bool manageOctomap(bender_arm_planning::ManageOctomap::Request &req, bender_arm_planning::ManageOctomap::Response &res);

  ros::ServiceServer service_;
};

}

// BENDER_PLANNING_MANAGE_OCTOMAP_CAPABILITY_
#endif 
