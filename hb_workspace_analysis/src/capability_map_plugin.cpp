/* 
* Get grasp positions from MongoDB
*
* Author: Rodrigo Munoz
*/
#include <hb_workspace_analysis/capability_map_plugin.h>


namespace move_group
{

/*  struct GraspPositionOrder
  {
    inline bool operator() (const bender_arm_planning::GraspPosition& p, const bender_arm_planning::GraspPosition& q)
    {
      return (p.quality > q.quality);
    }
  };*/

  CapabilityMapPlugin::CapabilityMapPlugin():
    MoveGroupCapability("CapabilityMapPlugin")
  {
  }

  void CapabilityMapPlugin::initialize()
  {
    // Grasp service
    grasp_service_ = root_node_handle_.advertiseService(CAPABILITY_MAP_PLUGIN_NAME,
                                                        &CapabilityMapPlugin::getCapabilityMapCb, this);

    try
    {
      // Make connection to DB
      db_.reset(new GraspStorageDb("workspace_analysis", "capability_map", "localhost", 27017, 5.0));
    }
    catch (const mongo_ros::DbConnectException& exception)
    {
      // Connection timeout
      ROS_ERROR("Connection timeout.");
      db_.reset(); // Make null pointer
    }
    // TODO Add parameter
    resolution_ = 0.01; // 1 cm
    search_factor_ = 1.0; // 0.5 cm
  }

  bool CapabilityMapPlugin::getCapabilityMapCb(hb_workspace_analysis::GetCapabilityMap::Request &req,
                                               hb_workspace_analysis::GetCapabilityMap::Response &res)
  {
    // Check db connection
    if (!db_)
      return false;

    ROS_INFO_STREAM("Number of elements: " << db_->count());
    /* --------------------------------------------------------------------------------
    * Get grasps from db
    */
    // Create query using object pose
    // X range (greater than or equal, less than or equal)
    const double x_gte = req.object.primitive_poses[0].position.x - resolution_*search_factor_;
    const double x_lte = req.object.primitive_poses[0].position.x + resolution_*search_factor_;
    // Y range (greater than or equal, less than or equal)
    const double y_gte = req.object.primitive_poses[0].position.y - resolution_*search_factor_;
    const double y_lte = req.object.primitive_poses[0].position.y + resolution_*search_factor_;
    // Z range (greater than or equal, less than or equal) using cylinder height
    const double z_gte = req.object.primitive_poses[0].position.z - resolution_*search_factor_ - req.object.primitives[0].dimensions[0]/2;
    const double z_lte = req.object.primitive_poses[0].position.z + resolution_*search_factor_ + req.object.primitives[0].dimensions[0]/2;
    ROS_INFO_STREAM("Searching at: x[" << x_gte << ", " << x_lte << "] y[" << y_gte << ", " << y_lte << "] z[" << z_gte << ", " << z_lte << "] ");
    mongo_ros::Query query = mongo_ros::Query()
        .append("x", mongo_ros::GTE, x_gte).append("x", mongo_ros::LTE, x_lte)
        .append("y", mongo_ros::GTE, y_gte).append("y", mongo_ros::LTE, y_lte)
        .append("z", mongo_ros::GTE, z_gte).append("z", mongo_ros::LTE, z_lte);
    // Send query with descending z
    std::vector<GraspStorageWithMetadataPtr> result = db_->pullAllResults(query, false, "z", false);
    if(result.empty())
    {
      ROS_WARN("Grasp not found.");
      return true;
    }

    /* --------------------------------------------------------------------------------
    * Save grasps in response
    */
    // Reserve space
    ROS_INFO_STREAM("Grasp result size: " << result.size());
    for(std::size_t i = 0; i < result.size(); ++i)
    {
      for(std::size_t j = 0; j < result[i]->grasp.size(); ++j)
        res.grasp.grasp.push_back(result[i]->grasp[j]);
    }

    ROS_INFO_STREAM("Grasp found: " << res.grasp.grasp.size());

    return true;

    /* --------------------------------------------------------------------------------
    * Filtro de colision para posiciones de pregrasp
    */
    /*    bool filter_pregrasp_collision = true;
    if (filter_pregrasp_collision)
    {
      for(std::size_t i = 0; i < res.grasps.size(); ++i)
      {
        filterPregraspCollision(res.grasps[i].grasp, req.group_name); // grasp esta al reves
      }
    }*/

    /* --------------------------------------------------------------------------------
    * Anadir indicador de calidad a los elementos
    */
    //addQuality(res.grasps, target_pose);
    /* --------------------------------------------------------------------------------
    * Ordenar elementos por calidad
    */
    //std::sort(res.grasps.begin(), res.grasps.end(), GraspPositionOrder());
    return true;
  } // getCapabilityMapCb




  bool CapabilityMapPlugin::filterPregraspCollision(std::vector<trajectory_msgs::JointTrajectoryPoint>& pregrasps, const std::string& group_name)
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
CLASS_LOADER_REGISTER_CLASS(move_group::CapabilityMapPlugin, move_group::MoveGroupCapability)
