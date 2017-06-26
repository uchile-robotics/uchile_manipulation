/*
* Get grasp positions from MongoDB
*
* Author: Rodrigo Munoz
*/
#include "hb_workspace_analysis/capability_map_plugin.h"


namespace move_group
{
  CapabilityMapPlugin::CapabilityMapPlugin():
    MoveGroupCapability("CapabilityMapPlugin")
  {
  }

  bool CapabilityMapPlugin::loadOptions()
  {


  }

  void CapabilityMapPlugin::initialize()
  {
    if (node_handle_.hasParam("capability_map"))
    {
      XmlRpc::XmlRpcValue raw_parameters;
      node_handle_.getParam("capability_map", raw_parameters);
      ROS_ASSERT(raw_parameters.getType() == XmlRpc::XmlRpcValue::TypeStruct);
      for(XmlRpc::XmlRpcValue::ValueStruct::iterator it = raw_parameters.begin(); it != raw_parameters.end(); ++it)
      {
        std::string capmap_name(it->first);
        ros::NodeHandle nh_capmap(node_handle_, "capability_map/" + capmap_name);
        hb_workspace_analysis::CapabilityMapOptions opt;
        opt.load(nh_capmap);
        GraspStorageDbPtr db;
        ROS_INFO_STREAM("Loading capability map: \n" << opt);
        // Create database connection
        try
        {
          // Make connection to DB
          db.reset(new GraspStorageDb(opt.db_name, opt.db_collection, opt.db_server, opt.db_port, 5.0));
        }
        catch (const mongo_ros::DbConnectException& exception)
        {
          // Connection timeout
          ROS_ERROR_STREAM("Connection timeout for capability map: \n" << opt);
          db.reset(); // Make null pointer
        }
        // Check for empty database
        if (db->count() == 0)
        {
          ROS_WARN_STREAM("Capability map for \"" << opt.group_name << "\" is empty.");
        }
        capmap_opt_.insert(std::make_pair<std::string, hb_workspace_analysis::CapabilityMapOptions>(opt.group_name, opt));
        db_.insert(std::make_pair<std::string, GraspStorageDbPtr>(opt.group_name, db));
      }
    }
    else
    {
      ROS_ERROR_STREAM("Parameters for CapabilityMapPlugin missing from parameter server. Searching in namespace: "
                           << node_handle_.getNamespace());
    }


    // Grasp service
    grasp_service_ = root_node_handle_.advertiseService(CAPABILITY_MAP_PLUGIN_NAME,
                                                        &CapabilityMapPlugin::getCapabilityMapCb, this);

  }

  bool CapabilityMapPlugin::getCapabilityMapCb(hb_workspace_analysis::GetCapabilityMap::Request &req,
                                               hb_workspace_analysis::GetCapabilityMap::Response &res)
  {
    // Check db connection
    DatabaseTable::iterator db_it = db_.find(req.group_name);
    CapabilityMapOptionsTable::iterator opt_it = capmap_opt_.find(req.group_name);
    if (db_it == db_.end() || opt_it == capmap_opt_.end())
    {
      ROS_ERROR_STREAM("Capability map not availaible for \"" << req.group_name << "\".");
      return false;
    }

    // Check for null pointer
    hb_workspace_analysis::CapabilityMapOptions& opt = capmap_opt_[req.group_name];
    GraspStorageDbPtr& db = db_[req.group_name];
    if (!db)
    {
      ROS_ERROR_STREAM("Capability couldn't be loaded for \"" << opt.group_name << "\".");
      return false;
    }


    ROS_INFO_STREAM("Number of elements: " << db->count());
    /* --------------------------------------------------------------------------------
    * Get grasps from db
    */
    // Time info
    ros::Time t0 = ros::Time::now();
    // Create query using object pose
    // X range (greater than or equal, less than or equal)
    const double x_gte = req.object.primitive_poses[0].position.x - opt.resolution*opt.search_factor;
    const double x_lte = req.object.primitive_poses[0].position.x + opt.resolution*opt.search_factor;
    // Y range (greater than or equal, less than or equal)
    const double y_gte = req.object.primitive_poses[0].position.y - opt.resolution*opt.search_factor;
    const double y_lte = req.object.primitive_poses[0].position.y + opt.resolution*opt.search_factor;
    // Z range (greater than or equal, less than or equal) using cylinder height
    const double z_gte = req.object.primitive_poses[0].position.z - opt.resolution*opt.search_factor - req.object.primitives[0].dimensions[0]/2;
    const double z_lte = req.object.primitive_poses[0].position.z + opt.resolution*opt.search_factor + req.object.primitives[0].dimensions[0]/2;
    ROS_INFO_STREAM("Searching at: x[" << x_gte << ", " << x_lte << "] y[" << y_gte << ", " << y_lte << "] z[" << z_gte << ", " << z_lte << "] ");
    mongo_ros::Query query = mongo_ros::Query()
        .append("x", mongo_ros::GTE, x_gte).append("x", mongo_ros::LTE, x_lte)
        .append("y", mongo_ros::GTE, y_gte).append("y", mongo_ros::LTE, y_lte)
        .append("z", mongo_ros::GTE, z_gte).append("z", mongo_ros::LTE, z_lte);
    // Send query with descending z
    std::vector<GraspStorageWithMetadataPtr> result = db->pullAllResults(query, false, "z", false);
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
    const robot_state::JointModelGroup* joint_model_group = rs.getJointModelGroup(opt.group_name);
    if (!joint_model_group)
    {
      ROS_ERROR_STREAM("Group '"<< opt.group_name << "' not found in model.");
      return false;
    }
    // Get joint names
    const std::vector<std::string>& joint_names = joint_model_group->getActiveJointModelNames();
    // Configure collision request
    collision_detection::CollisionRequest creq;
    creq.group_name = opt.group_name;
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
