/*
* Get grasp positions from MongoDB
*
* Author: Rodrigo Munoz
*/
#include "hb_workspace_analysis/capability_map_plugin.h"
#include <pcl_conversions/pcl_conversions.h>


namespace move_group
{
  CapabilityMapPlugin::CapabilityMapPlugin():
    MoveGroupCapability("CapabilityMapPlugin")
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

        capmap_opt_.insert(std::make_pair<std::string, hb_workspace_analysis::CapabilityMapOptions>(opt.group_name, opt));
        db_.insert(std::make_pair<std::string, GraspStorageDbPtr>(opt.group_name, db));
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

    pc_pub_ = root_node_handle_.advertise<sensor_msgs::PointCloud2>("best_base", 10);
    ROS_INFO_STREAM("Publishing on" << pc_pub_.getTopic());

    /* --------------------------------------------------------------------------------
    * Grasp filter
    */
    // Get robot state and construct grasp filter
    const robot_state::RobotState& robot_state = context_->planning_scene_monitor_->getPlanningScene()->getCurrentState();

    ROS_INFO_STREAM(robot_state.getRobotModel()->getJointModelGroup("l_arm")->getSolverInstance()->getTipFrame());

    grasp_filter_.reset(new hb_grasp_generator::GraspFilter(robot_state));

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

    if (req.object.primitive_poses.empty())
    {
      ROS_ERROR_STREAM("Request have empty object.");
      return false;
    }
    /* --------------------------------------------------------------------------------
    * Online generation
    */
    if (req.generate_online)
    {
      // Grasp vector
      std::vector<moveit_msgs::Grasp> possible_grasps;

      // Generate set of grasps for one object
      grasp_gen_[req.group_name]->generateGrasp(req.object.primitive_poses[0], possible_grasps);

      std::vector<trajectory_msgs::JointTrajectoryPoint> ik_solutions;
      std::string end_effector_parent_link = context_->planning_scene_monitor_->getPlanningScene()->getCurrentState().getRobotModel()->
          getJointModelGroup(req.group_name)->getSolverInstance()->getTipFrame();

      grasp_filter_->filterGrasps(possible_grasps,
                                  ik_solutions,
                                  true,
                                  end_effector_parent_link,
                                  req.group_name, 0.1);
      if (!possible_grasps.empty())
      {
        geometry_msgs::Point& object_pose = req.object.primitive_poses[0].position;
        ROS_INFO_STREAM("Saving grasp data on database " << object_pose.x << " y: " << object_pose.y <<
                                                         " z: " << object_pose.z);

        // Construct grasp storage
        hb_workspace_analysis::GraspStorage grasp_vector;
        grasp_vector.header.frame_id = context_->planning_scene_monitor_->getPlanningScene()->getCurrentState().getRobotModel()->
            getJointModelGroup(req.group_name)->getSolverInstance()->getBaseFrame();
        grasp_vector.pose = req.object.primitive_poses[0];
        grasp_vector.grasp.reserve(possible_grasps.size());
        for(std::size_t n = 0; n < possible_grasps.size(); ++n)
        {
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
        db->insert(grasp_vector, metadata);
      }

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


  bool CapabilityMapPlugin::getBestBasePoseCb(hb_workspace_analysis::GetBestBasePose::Request &req,
                                              hb_workspace_analysis::GetBestBasePose::Response &res)
  {
    /* --------------------------------------------------------------------------------
    * Check
    */
    // Check for empty request
    if (req.object.primitive_poses.empty())
    {
      ROS_ERROR_STREAM("Request have empty object.");
      return false;
    }
    /* --------------------------------------------------------------------------------
    * Create query
    */
    double x_length = 1.0;
    double y_length = 1.0;
    double z_range = 0.05;
    // Time info
    ros::Time t0 = ros::Time::now();
    // Create query using object pose
    // X range (greater than or equal, less than or equal)
    const double x_gte = req.object.primitive_poses[0].position.x - x_length/2;
    const double x_lte = req.object.primitive_poses[0].position.x + x_length/2;
    // Y range (greater than or equal, less than or equal)
    const double y_gte = req.object.primitive_poses[0].position.y - y_length/2;
    const double y_lte = req.object.primitive_poses[0].position.y + y_length/2;
    // Z range (greater than or equal, less than or equal) using cylinder height
    const double z_gte = req.object.primitive_poses[0].position.z - z_range/2;
    const double z_lte = req.object.primitive_poses[0].position.z + z_range/2;
    ROS_INFO_STREAM("Searching at: x[" << x_gte << ", " << x_lte << "] y[" << y_gte << ", " << y_lte << "] z[" << z_gte << ", " << z_lte << "] ");
    mongo_ros::Query query = mongo_ros::Query()
        .append("x", mongo_ros::GTE, x_gte).append("x", mongo_ros::LTE, x_lte)
        .append("y", mongo_ros::GTE, y_gte).append("y", mongo_ros::LTE, y_lte)
        .append("z", mongo_ros::GTE, z_gte).append("z", mongo_ros::LTE, z_lte);
    /* --------------------------------------------------------------------------------
    * Create grid
    */
    // Check db connection
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZHSV>);
    uint32_t max_grasp = 0UL;
    DatabaseTable::iterator db_it;
    for (db_it = db_.begin(); db_it !=db_.end(); db_it++)
    {
      if (!db_it->second)
      {
        ROS_WARN_STREAM("Capability couldn't be loaded for \"" << db_it->first << "\".");
        continue;
      }
      // Send query with descending z
      std::vector<GraspStorageWithMetadataPtr> result = db_it->second->pullAllResults(query, false, "z", false);
      // Process data using PCL
      union { float f; uint32_t i; } u;
      for(std::size_t i = 0; i < result.size(); ++i)
      {
        u.i = result[i]->grasp.size();
        max_grasp = std::max(max_grasp, u.i);
        // Create a point (use hue for size)
        pcl::PointXYZHSV point;
        point.x = result[i]->pose.position.x;
        point.y = result[i]->pose.position.y;
        point.z = result[i]->pose.position.z;
        point.h = u.f;
        point.s = 1.0f;
        point.v = 1.0f;
        // Add to point cloud
        cloud->points.push_back(point);
      }
    }
    sensor_msgs::PointCloud2::Ptr output(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*cloud, *output);

    // Publish the data
    output->header.stamp = ros::Time::now();
    output->header.frame_id = "/bender/base_link";
    pc_pub_.publish(output);

  }

}


#include <class_loader/class_loader.h>
CLASS_LOADER_REGISTER_CLASS(move_group::CapabilityMapPlugin, move_group::MoveGroupCapability)
