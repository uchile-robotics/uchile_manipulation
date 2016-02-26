/* 
* Permite obtener grasps precalculados
*
* Autor: Rodrigo Munoz
*/

#include <bender_arm_planning/capability_map_service.h>
#include <moveit_msgs/CollisionObject.h>
#include <shape_msgs/SolidPrimitive.h>
#include <geometry_msgs/PoseStamped.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>


namespace move_group
{

  struct GraspPositionOrder
  {
    inline bool operator() (const bender_arm_planning::GraspPosition& p, const bender_arm_planning::GraspPosition& q)
    {
      return (p.quality > q.quality);
    }
  };

  CapabilityMapService::CapabilityMapService():
    MoveGroupCapability("CapabilityMapService")
  {
  }

  void CapabilityMapService::initialize()
  {
    // Servicio grasp
    grasp_service_ = root_node_handle_.advertiseService(CAPABILITY_MAP_SERVICE_NAME, &CapabilityMapService::getGraspsCb, this);
    // Servicio distancia
    distance_service_ = root_node_handle_.advertiseService("base_distance", &CapabilityMapService::getBestDistanceCb, this);
    // Servicio ACM
    acm_service_ = root_node_handle_.advertiseService("acm_object", &CapabilityMapService::addObjectACMCb, this);

    // Obtener ruta archivo
    std::string file_name;
    if (root_node_handle_.hasParam("capmap_file"))
    {
      root_node_handle_.getParam("capmap_file", file_name);
      ROS_INFO("Using capability map file: %s", file_name.c_str());
    }
    else
    {
      ROS_WARN("Parameter 'capmap_file' missing from parameter server in namespace %s. Using default.", root_node_handle_.getNamespace().c_str());
      file_name = "cap_map.dat";
    }
    // Cargar archivo
    ROS_INFO("Loading capability map file %s", file_name.c_str());
    capability_map_ = capability_map::loadCapMap(file_name);
    if(!capability_map_)
    {
      ROS_ERROR("Capability map file not loaded.");
    }
    else
    {
      ROS_INFO_STREAM("\033[92m" << "Capability Map loaded. " << "\033[0m");
      z_res_ = capability_map_->getResolution('z');
      x_res_ = capability_map_->getResolution('x');
      ref_frame_ = capability_map_->getReferenceFrame();
    }
    
  }

  bool CapabilityMapService::addObjectACMCb(bender_arm_planning::AddObjectACM::Request &req, bender_arm_planning::AddObjectACM::Response &res)
  {
    
    ROS_INFO("Processing collision object %s", req.target.id.c_str());
    planning_scene_monitor::LockedPlanningSceneRW planning_scene(context_->planning_scene_monitor_);
    
    if(!req.touch_links.empty()){  
      collision_detection::AllowedCollisionMatrixPtr approach_grasp_acm(new collision_detection::AllowedCollisionMatrix(planning_scene->getAllowedCollisionMatrix()));
      approach_grasp_acm->setEntry(req.target.id, req.touch_links, true);
      planning_scene->getCurrentStateNonConst().update(); // hack to prevent bad transforms
    }
    planning_scene->processCollisionObjectMsg(req.target);
    context_->planning_scene_monitor_->triggerSceneUpdateEvent(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);
    
    return true;
  }



  bool CapabilityMapService::getGraspsCb(bender_arm_planning::CapabilityMapGrasp::Request &req, bender_arm_planning::CapabilityMapGrasp::Response &res)
  {
    /* --------------------------------------------------------------------------------
    * Chequear que el capability map se ha cargado correctamente
    */
    if(!capability_map_)
    {
      ROS_ERROR("Capability map file not loaded.");
      return false;
    }
    /* --------------------------------------------------------------------------------
    * Obtener parametros
    */
    geometry_msgs::PoseStamped target_pose;
    double target_height;
    if(!getTargetPose(req.target, target_pose, target_height))
    {
      ROS_ERROR("Error getting target pose");
      return true;
    }

    /* --------------------------------------------------------------------------------
    * Obtener grasps
    */
    int n = target_height/z_res_;
    // Posicion inferior del cilindro
    Eigen::Vector3d target(target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z - target_height/2);
    Eigen::Vector3d grasp_saved, error;
    
    std::vector<capability_map::GraspPoint> grasp_cm;
    
    for (unsigned int i = 0; i < n; ++i)
    {
      if (capability_map_->getGrasp(target, grasp_cm))
      {
        /*
        grasp_cm.back().getPosition(grasp_saved); // Ultima posicion guardada en cap_map
        error = grasp_saved - target;
        ROS_INFO_STREAM("Target " << target.transpose() 
          << " Saved " << grasp_saved.transpose() << " Error " << error.norm());
        */
      }
      target(2) += z_res_;
    }
    if (grasp_cm.empty())
    {
      ROS_WARN("No grasps generated.");
      return true;
    }
    ROS_INFO_STREAM("Found " << grasp_cm.size() << " grasps");
    /* --------------------------------------------------------------------------------
    * Anadir elementos a response
    */
    res.grasps.resize(grasp_cm.size());
    for(std::size_t i = 0; i < grasp_cm.size(); ++i)
    {
      grasp_cm[i].getGraspPositionMsg(res.grasps[i], ref_frame_);
    }
    /* --------------------------------------------------------------------------------
    * Filtro de colision para posiciones de pregrasp
    */
    bool filter_pregrasp_collision = true;
    if (filter_pregrasp_collision)
    {
      for(std::size_t i = 0; i < res.grasps.size(); ++i)
      {
        filterPregraspCollision(res.grasps[i].grasp, req.group_name); // grasp esta al reves
      }
    }

    /* --------------------------------------------------------------------------------
    * Anadir indicador de calidad a los elementos
    */
    addQuality(res.grasps, target_pose);
    /* --------------------------------------------------------------------------------
    * Ordenar elementos por calidad
    */
    std::sort(res.grasps.begin(), res.grasps.end(), GraspPositionOrder());
    return true;
  } // getGraspsCb



  void CapabilityMapService::addQuality(std::vector<bender_arm_planning::GraspPosition>& grasps, const geometry_msgs::PoseStamped& target_pose) const
  {
    for (unsigned int i = 0; i < grasps.size(); ++i)
    {
      double height = grasps[i].grasp_pose.pose.position.z * 10;// Altura de grasp
      double center_distance = height - target_pose.pose.position.z * 10; // Distancia del centro del objeto

      grasps[i].quality +=  height + center_distance;

    }
  }

  bool CapabilityMapService::getTargetPose(const moveit_msgs::CollisionObject& target, geometry_msgs::PoseStamped& target_pose, double& target_height) const
  {
    /* --------------------------------------------------------------------------------
    * Obtener geometrias
    * Obtener geometria cilindrica para generar grasps, se escoje la primera en el
    * arreglo object_target.primitives
    * @TODO: Manejo de distintas geometrias y combinacion de ellas
    *
    */
    if (target.primitives.empty())
    {
      ROS_ERROR("No primitive elements for grasp generator with object %s", target.id.c_str());
      return false;
    }
    // Obtener geometrias cilindricas
    bool find_grasp = false;
    for (std::size_t i = 0; i < target.primitives.size(); ++i)
    {
      if (target.primitives[i].type == shape_msgs::SolidPrimitive::CYLINDER)
      {
        // Completar campos
        target_pose.pose = target.primitive_poses[i];
        target_pose.header.frame_id = target.header.frame_id;
        target_height = target.primitives[i].dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT];
        find_grasp = true;
        break;
      }
    }
    if (!find_grasp)
    {
      ROS_ERROR("Can not find cylinder component in object %s", target.id.c_str());
      return false;
    }
    /* --------------------------------------------------------------------------------
    * Obtener pose con respecto a frame del capability map
    * Funcion performTransform se hereda de MoveGroupCapability
    */
    return performTransform(target_pose, ref_frame_);
  }

  bool CapabilityMapService::getBestDistanceCb(bender_arm_planning::CapabilityMapBestDistance::Request &req, bender_arm_planning::CapabilityMapBestDistance::Response &res)
  {
    /* --------------------------------------------------------------------------------
    * Chequear que el capability map se ha cargado correctamente
    */
    if(!capability_map_)
    {
      ROS_ERROR("Capability map file not loaded.");
      return false;
    }
    /* --------------------------------------------------------------------------------
    * Obtener parametros
    */
    geometry_msgs::PoseStamped target_pose;
    double target_height;
    if(!getTargetPose(req.target, target_pose, target_height))
    {
      ROS_ERROR("Error getting target pose");
      return true;
    }
    /* --------------------------------------------------------------------------------
    * Obtener mejor grasps a lo largo del eje x
    */
    int n = req.range/x_res_; // Numero de grasps en el espacio x seleccionado
    int m = target_height/z_res_;
    // Posicion mas cercana en x
    ROS_DEBUG("Real target %.2f %.2f %.2f", target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z);
    Eigen::Vector3d target(target_pose.pose.position.x - req.range/2, target_pose.pose.position.y, target_pose.pose.position.z - target_height/2);
    // Objetos para almecenar mejor posicion
    capability_map::GraspPoint gp;
    Eigen::Vector3d best_position;
    std::size_t best_grasp_size = 0, current_size = 0;
    for (unsigned int i = 0; i < n; ++i) // Rango en x
    {
      for (unsigned int j = 0; j < m; ++j) // Rango en z
      {
        if (capability_map_->getGrasp(target, gp))
        {
          current_size += gp.size();
        }
        target(2) += z_res_;
      }
      target(2) = target_pose.pose.position.z - target_height/2;

      if (current_size > best_grasp_size)
      {
        // Actualizar valores de mejor posicion
        best_grasp_size = current_size;
        ROS_DEBUG_STREAM("Grasp x:" << target(0) << " with "<< best_grasp_size);
        gp.getPosition(best_position);
      }
      // Actualizar posicion en x
      target(0) += x_res_;
      current_size = 0;
    }
    if (best_grasp_size == 0)
    {
      ROS_WARN("No grasps en range.");
      return true;
    }
    /* --------------------------------------------------------------------------------
    * Obtener distancia
    */
    res.distance = target_pose.pose.position.x - best_position(0);
    ROS_INFO_STREAM("Best distance " << res.distance);
    return true;
  } // getBestDistanceCb


  bool CapabilityMapService::filterPregraspCollision(std::vector<trajectory_msgs::JointTrajectoryPoint>& pregrasps, const std::string& group_name)
  {
    // Obtener Planning scene
    planning_scene_monitor::LockedPlanningSceneRO ls(context_->planning_scene_monitor_);
    robot_state::RobotState rs = ls->getCurrentState();

    const robot_state::JointModelGroup* joint_model_group = rs.getJointModelGroup(group_name);
    if (!joint_model_group)
    {
      ROS_ERROR("Group '%s' not found in model.", group_name.c_str());
      return false;
    }
    // Obtener nombres de joints
    const std::vector<std::string>& joint_names = joint_model_group->getActiveJointModelNames();
    // Configurar request de colision
    collision_detection::CollisionRequest creq;
    creq.group_name = group_name;
    creq.cost = false;
    creq.contacts = false;
    creq.max_contacts = 1;
    creq.max_cost_sources = 1;
    collision_detection::CollisionResult cres;

    // Obtener indices de posiciones de pregrasp en colision
    std::vector<std::size_t> collision_pos;
    collision_pos.reserve(pregrasps.size());
    for(std::size_t i = 0; i < pregrasps.size(); ++i)
    {
      rs.setVariablePositions(joint_names, pregrasps[i].positions);
      ls->checkCollision(creq, cres, rs);
      if (cres.collision)
      {
        collision_pos.push_back(i);
      }
    }
    
    if (collision_pos.empty())
    {
      ROS_DEBUG("All grasp are collision free.");
      return true;
    }

    ROS_DEBUG_STREAM("Found " << collision_pos.size() << " grasps in collision.");

    // Remover elementos, debe realizarse de atras hace adelante
    for (std::vector<std::size_t>::reverse_iterator i = collision_pos.rbegin(); i != collision_pos.rend(); ++i)
    {
      pregrasps.erase(pregrasps.begin() + *i);
    }
    return true;

  } // filterPregraspCollision


}


#include <class_loader/class_loader.h>
CLASS_LOADER_REGISTER_CLASS(move_group::CapabilityMapService, move_group::MoveGroupCapability)
