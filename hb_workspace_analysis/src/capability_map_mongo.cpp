#include <mongo_ros/message_collection.h>
#include <hb_workspace_analysis/GraspStorage.h>
#include <hb_grasp_generator/grasp_filter.h>
#include <hb_grasp_generator/grasp_options.h>
#include <hb_grasp_generator/grasp_generator.h>

typedef mongo_ros::MessageWithMetadata<hb_workspace_analysis::GraspStorage> GraspStorageWithMetadata;

int main (int argc, char** argv)
{
  ros::init(argc, argv, "client_test");
  ros::NodeHandle nh;

  // Symbols used in queries to denote binary predicates for < and >
  using mongo_ros::LT;
  using mongo_ros::GT;

  // Clear existing data if any
  mongo_ros::dropDatabase("workspace_analysis", "localhost", 27017, 5.0);

  // Set up db
  mongo_ros::MessageCollection<hb_workspace_analysis::GraspStorage> coll("workspace_analysis", "capability_map", "localhost", 27017, 5.0);

  // Arrange to index on metadata field 'z'
  coll.ensureIndex("z");

  // Get arm info from param server
  ros::NodeHandle nh_("~");
  std::string planning_group_name_;
  nh_.param("arm", planning_group_name_, std::string("l_arm"));
  std::string ee_group_name_;
  nh_.param("ee_name", ee_group_name_, std::string("l_gripper"));
  ROS_INFO_STREAM_NAMED("test","Planning Group: " << planning_group_name_);
  ROS_INFO_STREAM_NAMED("test","End Effector: " << ee_group_name_);

  // ---------------------------------------------------------------------------------------------
  // Load planning scene to share
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));

  // ---------------------------------------------------------------------------------------------
  // Load grasp options
  hb_grasp_generator::GraspOptions opt;
  opt.load(nh_, ee_group_name_);
  ros::NodeHandle grasp_nh(nh_, ee_group_name_);
  hb_grasp_generator::CylindricalGraspGeneratorPtr simple_grasps_(new hb_grasp_generator::CylindricalGraspGenerator(grasp_nh, opt));

  // ---------------------------------------------------------------------------------------------
  // Load Grasp filter
  const robot_state::RobotState& robot_state = planning_scene_monitor_->getPlanningScene()->getCurrentState();
  hb_grasp_generator::GraspFilterPtr grasp_filter_(new hb_grasp_generator::GraspFilter(robot_state) );

  // ---------------------------------------------------------------------------------------------
  // Grasp position
  geometry_msgs::Pose object_pose;
  // Default orientation
  object_pose.orientation.x = 0.0;
  object_pose.orientation.y = 0.0;
  object_pose.orientation.z = 0.0;
  object_pose.orientation.w = 1.0;

  std::vector<moveit_msgs::Grasp> possible_grasps;
  std::vector<trajectory_msgs::JointTrajectoryPoint> ik_solutions; // save each grasps ik solution for visualization

  // ---------------------------------------------------------------------------------------------
  // Get params for capability map
  std::vector<double> init_pos, final_pos;
  // Get init position
  if (nh_.hasParam("init_pos"))
  {
    nh_.getParam("init_pos", init_pos);
    if (init_pos.size() != 3)
      throw std::runtime_error("Parameter 'init_pos' must have length 3.");
    ROS_INFO_STREAM("Using 'init_pos' (x: " << init_pos[0] << " y: "  << init_pos[1] << " z: " << init_pos[2] << ").");
  }
  else
  {
    init_pos.push_back(0.2);
    init_pos.push_back(0.0);
    init_pos.push_back(0.3);
    ROS_WARN_STREAM("Parameter 'init_pos' missing from parameter server. Using default (x: " << init_pos[0] <<
                                                                                             " y: "  << init_pos[1] << " z: " << init_pos[2] << ").");
  }
  // Get final position
  if (nh_.hasParam("final_pos"))
  {
    nh_.getParam("final_pos", final_pos);
    if (final_pos.size() != 3)
      throw std::runtime_error("Parameter 'final_pos' must have length 3.");
    ROS_INFO_STREAM("Using 'final_pos' (x: " << final_pos[0] << " y: "  << final_pos[1] << " z: " << final_pos[2] << ").");
  }
  else
  {
    final_pos.push_back(0.8);
    final_pos.push_back(0.5);
    final_pos.push_back(1.2);
    ROS_WARN_STREAM("Parameter 'final_pos' missing from parameter server. Using default (x: " << final_pos[0] <<
                                                                                              " y: "  << final_pos[1] << " z: " << final_pos[2] << ").");
  }
  // Get resolution, default value at 1cm
  double resolution = 0.01;
  nh_.param<double>("resolution", resolution, 0.01);
  ROS_INFO_STREAM("Resolution: " << resolution << " m");
  // Get resolution, default value at 1cm
  double ik_timeout = 0.1;
  nh_.param<double>("ik_timeout", ik_timeout, 0.1);
  ROS_INFO_STREAM("IK timeout: " << ik_timeout << " s");
  // Filter grasp option, must be true
  bool filter_pregrasps = true;
  // TODO Make percentage indicator

  for(std::size_t i = 0; init_pos[0]+i*resolution < final_pos[0]; ++i)
  {
    for(std::size_t j = 0; init_pos[1]+j*resolution < final_pos[1]; ++j)
    {
      for(std::size_t k = 0; init_pos[2]+k*resolution < final_pos[2]; ++k)
      {
        // Check ROS state
        if (!ros::ok())
          break;
        // Update object position
        object_pose.position.x = init_pos[0]+i*resolution;
        object_pose.position.y = init_pos[1]+j*resolution;
        object_pose.position.z = init_pos[2]+k*resolution;
        ROS_INFO_STREAM("Object position: x: " << object_pose.position.x << " y: " << object_pose.position.y <<
                                               " z: " << object_pose.position.z);
        // Clear last solution
        possible_grasps.clear();
        ik_solutions.clear();
        // Generate set of grasps for one object
        simple_grasps_->generateGrasp(object_pose, possible_grasps);
        // Apply grasp filter
        grasp_filter_->filterGrasps(possible_grasps,
                                    ik_solutions,
                                    filter_pregrasps,
                                    opt.end_effector_parent_link,
                                    planning_group_name_, ik_timeout);

        // Check for available grasp
        if (possible_grasps.empty())
          continue;
        // Construct grasp storage
        hb_workspace_analysis::GraspStorage grasp_vector;
        grasp_vector.header.frame_id = opt.base_link;
        grasp_vector.pose = object_pose;
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
        coll.insert(grasp_vector, metadata);
        ROS_INFO_STREAM("Collection count: " << coll.count());
      }
    }
  }
}
