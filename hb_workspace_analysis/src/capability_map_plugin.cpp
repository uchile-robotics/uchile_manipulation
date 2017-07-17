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

  void CapabilityMapPlugin::initialize()
  {
    // Get robot URDF from context
    urdf_ = context_->planning_scene_monitor_->getRobotModelLoader()->getURDF();
    // TF client from plugin context
    tf_ = context_->planning_scene_monitor_->getTFClient();

    /* --------------------------------------------------------------------------------
    * Get parameters
    */
    if (node_handle_.hasParam("capability_map"))
    {
      XmlRpc::XmlRpcValue raw_parameters;
      node_handle_.getParam("capability_map", raw_parameters);
      ROS_ASSERT(raw_parameters.getType() == XmlRpc::XmlRpcValue::TypeStruct);
      // Get robot state for construct grasp filter
      const robot_state::RobotState& robot_state = context_->planning_scene_monitor_->getPlanningScene()->getCurrentState();
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
        if (db && db->count() == 0)
        {
          ROS_WARN_STREAM("Capability map for \"" << opt.group_name << "\" is empty.");
        }

        /* --------------------------------------------------------------------------------
        * Get grasps generator
        */
        if (node_handle_.hasParam("grasp_generator"))
        {
          ros::NodeHandle grasp_nh(node_handle_, "grasp_generator");
          hb_grasp_generator::GraspOptions grasp_opt;
          grasp_opt.load(grasp_nh, opt.group_name);
          ros::NodeHandle grasp_gren_nh(grasp_nh, "cylindrical_grasp_generator");
          hb_grasp_generator::CylindricalGraspGeneratorPtr grasp_gen(
              new hb_grasp_generator::CylindricalGraspGenerator(grasp_gren_nh, grasp_opt));
          grasp_gen_.insert(std::make_pair<std::string, hb_grasp_generator::CylindricalGraspGeneratorPtr>(opt.group_name, grasp_gen));
        }
        else
        {
          ROS_ERROR_STREAM("Parameters \"grap_generator\" for CapabilityMapPlugin missing from parameter server. Searching in namespace: "
                               << node_handle_.getNamespace());
        }

        /* --------------------------------------------------------------------------------
        * Grasp filter
        */
        hb_grasp_generator::GraspFilterPtr filter(new hb_grasp_generator::GraspFilter(robot_state, opt.group_name));

        /* --------------------------------------------------------------------------------
        * Create tables
        */
        capmap_opt_.insert(std::make_pair<std::string, hb_workspace_analysis::CapabilityMapOptions>(opt.group_name, opt));
        db_.insert(std::make_pair<std::string, GraspStorageDbPtr>(opt.group_name, db));
        grasp_filter_.insert(std::make_pair<std::string, hb_grasp_generator::GraspFilterPtr>(opt.group_name, filter));
      }
    }
    else
    {
      ROS_ERROR_STREAM("Parameters \"capability_map\" for CapabilityMapPlugin missing from parameter server. Searching in namespace: "
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
    GraspFilterTable::iterator gf_it = grasp_filter_.find(req.group_name);
    if (db_it == db_.end() || opt_it == capmap_opt_.end() || gf_it == grasp_filter_.end())
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

    if (req.object.primitive_poses.empty())
    {
      ROS_ERROR_STREAM("Request have empty object.");
      return false;
    }

    // Main object target pose
    geometry_msgs::Pose target_pose = req.object.primitive_poses[0];
    // Get Planning scene
    planning_scene_monitor::LockedPlanningSceneRO ls(context_->planning_scene_monitor_);
    robot_state::RobotState rs = ls->getCurrentState();
    const robot_state::JointModelGroup* joint_model_group = rs.getJointModelGroup(opt.group_name);
    if (!joint_model_group)
    {
      ROS_ERROR_STREAM("Group '"<< opt.group_name << "' not found in model.");
      return false;
    }
    const std::vector<std::string>& joint_names = joint_model_group->getActiveJointModelNames();
    // Time info
    ros::Time t0 = ros::Time::now();
    /* --------------------------------------------------------------------------------
    * Online generation
    */
    if (req.generate_online)
    {
      // Grasp vector
      std::vector<moveit_msgs::Grasp> possible_grasps;

      /* --------------------------------------------------------------------------------
      * Transform to kinematic base frame
      */
      const std::string& ik_frame = grasp_filter_[req.group_name]->getBaseFrame();
      const std::string& grasp_frame = req.object.header.frame_id;
      if (!moveit::core::Transforms::sameFrame(ik_frame, grasp_frame))
      {
        ROS_INFO_STREAM("IK frame must be the same than Grasp frame, using TF for transform.");
        // Object pose to TF stamped pose
        tf::Stamped<tf::Pose> original_pose_tf, target_pose_tf;
        tf::poseMsgToTF(req.object.primitive_poses[0], original_pose_tf);
        original_pose_tf.stamp_ = req.object.header.stamp;
        original_pose_tf.frame_id_ = grasp_frame;
        // Transform pose
        try
        {
          this->tf_->transformPose(ik_frame, original_pose_tf, target_pose_tf);
        }
        catch(tf::LookupException& ex)
        {
          ROS_ERROR_STREAM("LookupException in transform \""<< grasp_frame << "\" to \"" << ik_frame << "\"");
          ROS_ERROR_STREAM(ex.what());
          return false;
        }
        catch(tf::ExtrapolationException& ex)
        {
          ROS_ERROR_STREAM("ExtrapolationException in transform \""<< grasp_frame << "\" to \"" << ik_frame << "\"");
          ROS_ERROR_STREAM(ex.what());
          return false;
        }
        catch(tf::ConnectivityException& ex)
        {
          ROS_ERROR_STREAM("ConnectivityException in transform \""<< grasp_frame << "\" to \"" << ik_frame << "\"");
          ROS_ERROR_STREAM(ex.what());
          return false;
        }
        // Convert to geometry message
        tf::poseTFToMsg(target_pose_tf, target_pose);
        ROS_INFO_STREAM("Grasp positon [" << target_pose.position.x << ", " << target_pose.position.y << ", " <<
                                          target_pose.position.z << "] with frame \"" << ik_frame << "\"");
      }
      // Generate set of grasps for one object
      grasp_gen_[req.group_name]->generateGrasp(target_pose, possible_grasps);

      std::vector<trajectory_msgs::JointTrajectoryPoint> ik_solutions;
      std::string end_effector_parent_link = context_->planning_scene_monitor_->getPlanningScene()->getCurrentState().getRobotModel()->
          getJointModelGroup(req.group_name)->getSolverInstance()->getTipFrame();

      grasp_filter_[req.group_name]->filterGrasps(possible_grasps, ik_solutions, true, 0.1);
      if (!possible_grasps.empty())
      {
        ROS_INFO_STREAM("Performing self collision check");
        // Construct grasp storage
        hb_workspace_analysis::GraspStorage grasp_vector;
        grasp_vector.header.frame_id = context_->planning_scene_monitor_->getPlanningScene()->getCurrentState().getRobotModel()->
            getJointModelGroup(req.group_name)->getSolverInstance()->getBaseFrame();
        grasp_vector.pose = req.object.primitive_poses[0];
        grasp_vector.grasp.reserve(possible_grasps.size());
        for(std::size_t n = 0; n < possible_grasps.size(); ++n)
        {
          // Self collision check for grasp and pre grasp positions
          bool collision = checkSelfCollision(ls, req.group_name, joint_names, ik_solutions[2*n].positions, rs) &&
              checkSelfCollision(ls, req.group_name, joint_names, ik_solutions[2*n+1].positions, rs);
          if (collision)
          {
            // Skip this grasp
            continue;
          }
          hb_workspace_analysis::GraspPoint point;
          point.grasp = possible_grasps[n];
          point.pregrasp_position = ik_solutions[2*n];
          point.grasp_position = ik_solutions[2*n+1];
          grasp_vector.grasp.push_back(point);
        }
        // Create metadata, this data is used for queries
        mongo_ros::Metadata metadata("x", grasp_vector.pose.position.x, "y", grasp_vector.pose.position.y,
                                     "z", grasp_vector.pose.position.z);
        // Insert data in db
        ROS_INFO_STREAM("Saving grasp data on database [" << target_pose.position.x << ", " << target_pose.position.y <<
                                                          ", " << target_pose.position.z << "]");
        db->insert(grasp_vector, metadata);
      }
    }

    ROS_INFO_STREAM("Number of elements: " << db->count());
    /* --------------------------------------------------------------------------------
    * Get grasps from db
    */
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
    // Configure collision request
    for(std::size_t i = 0; i < result.size(); ++i)
    {
      for(std::size_t j = 0; j < result[i]->grasp.size(); ++j)
      {
        // Collision filter for pregrasp position
        if(filter_pregrasp)
        {
          bool collision = checkCollision(ls, req.group_name, joint_names, result[i]->grasp[j].pregrasp_position.positions, rs);
          if (collision)
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

  bool CapabilityMapPlugin::checkSelfCollision(planning_scene_monitor::LockedPlanningSceneRO& ls,
                                               const std::string& group_name, const std::vector<std::string>& name,
                                               const std::vector<double>& position, robot_state::RobotState& robot_state)
  {
    // Configure collision request
    collision_detection::CollisionRequest creq;
    creq.group_name = group_name;
    creq.cost = false;
    creq.contacts = false;
    creq.max_contacts = 1;
    creq.max_cost_sources = 1;
    collision_detection::CollisionResult cres;
    // Index of pregrasp positions on collision
    std::vector<std::size_t> collision_pos;
    robot_state.setVariablePositions(name, position);
    ls->checkSelfCollision(creq, cres, robot_state);
    return cres.collision;
  }

  bool CapabilityMapPlugin::checkCollision(planning_scene_monitor::LockedPlanningSceneRO& ls,
                                           const std::string& group_name, const std::vector<std::string>& name,
                                           const std::vector<double>& position, robot_state::RobotState& robot_state)
  {
    // Configure collision request
    collision_detection::CollisionRequest creq;
    creq.group_name = group_name;
    creq.cost = false;
    creq.contacts = false;
    creq.max_contacts = 1;
    creq.max_cost_sources = 1;
    collision_detection::CollisionResult cres;
    // Index of pregrasp positions on collision
    std::vector<std::size_t> collision_pos;
    robot_state.setVariablePositions(name, position);
    ls->checkCollision(creq, cres, robot_state);
    return cres.collision;
  }
}


#include <class_loader/class_loader.h>
CLASS_LOADER_REGISTER_CLASS(move_group::CapabilityMapPlugin, move_group::MoveGroupCapability)
