/* 
* Get grasp positions from MongoDB
*
* Author: Rodrigo Munoz
*/
#include <hb_workspace_analysis/capability_map_plugin.h>


namespace move_group
{
  CapabilityMapPlugin::CapabilityMapPlugin():
    MoveGroupCapability("CapabilityMapPlugin")
  {
  }

  bool CapabilityMapPlugin::loadOptions()
  {
    ros::NodeHandle nh_capmap(node_handle_, "capability_map");
    nh_capmap.param<double>("resolution", resolution_, 0.01);
    nh_capmap.param<double>("search_factor", search_factor_, 1.0);
    nh_capmap.param<std::string>("group_name", group_name_, "arm");
    ROS_INFO_STREAM("Capability map for \"" << group_name_ << "\" using resolution " << resolution_ << " and search factor " << search_factor_);
    // Nodehandle for database parameters
    ros::NodeHandle nh_database(nh_capmap, "database");
    nh_database.param<std::string>("server", db_server_, "localhost");
    // Get MongoDB port
    nh_database.param<int>("port", db_port_, 27017);
    ROS_INFO_STREAM("Using MongoDB server " << db_server_ << ":" << db_port_);
    // Get database name
    nh_database.param<std::string>("name", db_name_, "workspace_analysis");
    // Get database name
    nh_database.param<std::string>("collection", collection_name_, "capability_map");
    ROS_INFO_STREAM("Collection \"" << collection_name_ << "\" in database \"" << db_name_ << "\"");
    return true;
  }

  void CapabilityMapPlugin::initialize()
  {
    loadOptions();
    // Grasp service
    grasp_service_ = root_node_handle_.advertiseService(CAPABILITY_MAP_PLUGIN_NAME,
                                                        &CapabilityMapPlugin::getCapabilityMapCb, this);
    try
    {
      // Make connection to DB
      db_.reset(new GraspStorageDb(collection_name_, db_name_, db_server_, db_port_, 5.0));
    }
    catch (const mongo_ros::DbConnectException& exception)
    {
      // Connection timeout
      ROS_ERROR("Connection timeout.");
      db_.reset(); // Make null pointer
    }
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
    // Time info
    ros::Time t0 = ros::Time::now();
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
    ROS_INFO_STREAM("Grasp result size: " << result.size());
    /* --------------------------------------------------------------------------------
    * Filter and save grasps in response
    */
    bool filter_pregrasp = true;
    // Get Planning scene
    planning_scene_monitor::LockedPlanningSceneRO ls(context_->planning_scene_monitor_);
    robot_state::RobotState rs = ls->getCurrentState();
    const robot_state::JointModelGroup* joint_model_group = rs.getJointModelGroup(group_name_);
    if (!joint_model_group)
    {
      ROS_ERROR_STREAM("Group '"<< group_name_ << "' not found in model.");
      return false;
    }
    // Get joint names
    const std::vector<std::string>& joint_names = joint_model_group->getActiveJointModelNames();
    // Configure collision request
    collision_detection::CollisionRequest creq;
    creq.group_name = group_name_;
    creq.cost = false;
    creq.contacts = false;
    creq.max_contacts = 1;
    creq.max_cost_sources = 1;
    collision_detection::CollisionResult cres;
    // Index of pregrasp positions on collision
    std::vector<std::size_t> collision_pos;
    for(std::size_t i = 0; i < result.size(); ++i)
    {
      for(std::size_t j = 0; j < result[i]->grasp.size(); ++j)
      {
        // Collision filter for pregrasp position
        if(filter_pregrasp)
        {
          rs.setVariablePositions(joint_names, result[i]->grasp[j].pregrasp_position.positions);
          ls->checkCollision(creq, cres, rs);
          if (cres.collision)
          {
            // If we found a collision we skip the grasp
            continue;
          }
        }
        // Add grasp
        res.grasp.grasp.push_back(result[i]->grasp[j]);
      }
    }
    ros::Duration dt = ros::Time::now() - t0;
    ROS_INFO_STREAM("Grasp without collision: " << res.grasp.grasp.size());
    ROS_INFO_STREAM("Time:  " << dt.toSec());
    return true;
  } // getCapabilityMapCb

}


#include <class_loader/class_loader.h>
CLASS_LOADER_REGISTER_CLASS(move_group::CapabilityMapPlugin, move_group::MoveGroupCapability)
