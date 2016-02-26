/* 
* Permite obtener grasps precalculados
*
* Autor: Rodrigo Munoz
*/

#include <bender_arm_planning/capmap_service.h>
#include <moveit_msgs/CollisionObject.h>
#include <shape_msgs/SolidPrimitive.h>
#include <geometry_msgs/PoseStamped.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>


namespace bender_arm_planning
{

  struct GraspPositionOrder
  {
    inline bool operator() (const GraspPosition& p, const GraspPosition& q)
    {
      return (p.quality > q.quality);
    }
  };

  CapabilityMapService::CapabilityMapService(const ros::NodeHandle& nh):
  nh_(nh)
  {
    // Servicio grasp
    grasp_service_ = nh_.advertiseService(CAPABILITY_MAP_SERVICE_NAME, &CapabilityMapService::getGraspsCb, this);
    // Servicio distancia
    distance_service_ = nh_.advertiseService("base_distance", &CapabilityMapService::getBestDistanceCb, this);
    // Obtener ruta archivo
    std::string file_name;
    if (nh_.hasParam("capmap_file"))
    {
      nh_.getParam("capmap_file", file_name);
      ROS_INFO("Using capability map file: %s", file_name.c_str());
    }
    else
    {
      ROS_WARN("Parameter 'capmap_file' missing from parameter server in namespace %s. Using default.", nh_.getNamespace().c_str());
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
      z_res_ = capability_map_->getResolution('z');
      x_res_ = capability_map_->getResolution('x');
      ref_frame_ = capability_map_->getReferenceFrame();
    }
  }

 
  bool CapabilityMapService::getGraspsCb(CapabilityMapGrasp::Request &req, CapabilityMapGrasp::Response &res)
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
    
    ROS_INFO("Getting grasps...");
    for (unsigned int i = 0; i < n; ++i)
    {
      if (capability_map_->getGrasp(target, grasp_cm))
      {
        grasp_cm.back().getPosition(grasp_saved); // Ultima posicion guardada en cap_map
        error = grasp_saved - target;
        //ROS_INFO_STREAM("Target " << target.transpose() 
          //<< " Saved " << grasp_saved.transpose() << " Error " << error.norm());
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
    ROS_INFO("Creating grasps messages...");
    res.grasps.resize(grasp_cm.size());
    for(unsigned int i = 0; i < grasp_cm.size(); ++i)
    {
      grasp_cm[i].getGraspPositionMsg(res.grasps[i], ref_frame_);
    }

    /* --------------------------------------------------------------------------------
    * Anadir indicador de calidad a los elementos
    */
    ROS_INFO("Adding quality to grasps messages...");
    addQuality(res.grasps, target_pose);
    /* --------------------------------------------------------------------------------
    * Ordenar elementos por calidad
    */
    ROS_INFO("Sorting grasps by quality...");
    std::sort(res.grasps.begin(), res.grasps.end(), GraspPositionOrder());
    return true;
  } // getGraspsCb



  void CapabilityMapService::addQuality(std::vector<GraspPosition>& grasps, const geometry_msgs::PoseStamped& target_pose) const
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
    return true;
    //return performTransform(target_pose, ref_frame_);
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
    //ROS_INFO("Real target %.2f %.2f %.2f", target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z);
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
          //ROS_INFO_STREAM("Grasp for " << target.transpose());
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
  }


}

int main(int argc, char** argv) 
{
  using namespace bender_arm_planning;

  ros::init(argc, argv, "capmap_service");
  ros::NodeHandle node_handle;

  CapabilityMapService capmap(node_handle);

  ros::spin();
  
  
  return 0;
}
