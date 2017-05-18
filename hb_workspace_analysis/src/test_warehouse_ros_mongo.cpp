#include <hb_workspace_analysis/test_mongo_helpers.h>
#include <mongo_ros/message_collection.h>
#include <hb_workspace_analysis/GraspStorage.h>
#include <hb_grasp_generator/grasp_filter.h>
#include <hb_grasp_generator/grasp_options.h>
#include <hb_grasp_generator/grasp_generator.h>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

namespace gm=geometry_msgs;
namespace hb_wa=hb_workspace_analysis;

using std::vector;
using std::string;
using std::cout;

typedef mongo_ros::MessageWithMetadata<gm::Pose> PoseWithMetadata;
typedef PoseWithMetadata::ConstPtr PoseMetaPtr;

// Helper function that creates metadata for a message.
// Here we'll use the x and y position, as well as a 'name'
// field that isn't part of the original message.
mongo_ros::Metadata makeMetadata (const gm::Pose& p, const string& v)
{
  return mongo_ros::Metadata("x", p.position.x, "y", p.position.y, "name", v);
}

mongo_ros::Metadata makeMetadata(const hb_workspace_analysis::GraspStorage& grasp)
{
    return mongo_ros::Metadata("x", grasp.pose.position.x, "y", grasp.pose.position.y,
                               "z", grasp.pose.position.z);
}

void test()
{
  // Symbols used in queries to denote binary predicates for < and >
  // Note that equality is the default, so we don't need a symbol for it
  using mongo_ros::LT;
  using mongo_ros::GT;

  // Clear existing data if any
  mongo_ros::dropDatabase("capability_map", "localhost", 27017, 5.0);
  
  // Set up db
  mongo_ros::MessageCollection<hb_workspace_analysis::GraspStorage> coll("workspace_analysis", "localhost",
                                       27017, 5.0);

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

  // Get joint model groups
  const robot_model::JointModelGroup
          *ee_jmg = planning_scene_monitor_->getRobotModel()->getJointModelGroup(ee_group_name_);
  const robot_model::JointModelGroup
          *arm_jmg = planning_scene_monitor_->getRobotModel()->getJointModelGroup(planning_group_name_);

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
  // Test grasp position
  geometry_msgs::Pose object_pose;
  object_pose.position.x = 0.50;
  object_pose.position.y = 0.29;
  object_pose.position.z = 0.60;

  object_pose.orientation.x = 0.0;
  object_pose.orientation.y = 0.0;
  object_pose.orientation.z = 0.0;
  object_pose.orientation.w = 1.0;

  std::vector<moveit_msgs::Grasp> possible_grasps;
  std::vector<trajectory_msgs::JointTrajectoryPoint> ik_solutions; // save each grasps ik solution for visualization

  ROS_INFO_STREAM_NAMED("test", "Adding object");

  possible_grasps.clear();
  ik_solutions.clear();

  // Generate set of grasps for one object
  simple_grasps_->generateGrasp(object_pose, possible_grasps);

  // Apply grasp filter
  bool filter_pregrasps = true;
  double ik_timeout = 0.1;
  grasp_filter_->filterGrasps(possible_grasps,
                              ik_solutions,
                              filter_pregrasps,
                              opt.end_effector_parent_link,
                              planning_group_name_, ik_timeout);

  // Construct grasp storage
  hb_workspace_analysis::GraspStorage grasp_vector;
  grasp_vector.header.frame_id = opt.base_link;
  grasp_vector.pose = object_pose;
  grasp_vector.grasp.reserve(possible_grasps.size());
  for(std::size_t i = 0; i < possible_grasps.size(); ++i)
  {
      hb_workspace_analysis::GraspPoint point;
      point.grasp = possible_grasps[i];
      point.pregrasp_position = ik_solutions[2*i];
      point.grasp_position = ik_solutions[2*i+1];
      grasp_vector.grasp.push_back(point);
  }

  coll.insert(grasp_vector, makeMetadata(grasp_vector));
  ROS_INFO_STREAM("Collection count: " << coll.count());

  // Add some poses and metadata
/*  const gm::Pose p1 = makePose(24, 42, 0);
  const gm::Pose p2 = makePose(10, 532, 3);
  const gm::Pose p3 = makePose(53, 22, 5);
  const gm::Pose p4 = makePose(22, -5, 33);
  coll.insert(p1, makeMetadata(p1, "bar"));
  coll.insert(p2, makeMetadata(p2, "baz"));
  coll.insert(p3, makeMetadata(p3, "qux"));
  coll.insert(p1, makeMetadata(p1, "oof"));
  coll.insert(p4, makeMetadata(p4, "ooof"));*/
  //EXPECT_EQ(5u, coll.count());

  // Simple query: find the pose with name 'qux' and return just its metadata
  // Since we're doing an equality check, we don't explicitly specify a predicate
  /*vector<PoseMetaPtr> res = coll.pullAllResults(mongo_ros::Query("name", "qux"), true);*/
  //EXPECT_EQ(1u, res.size());
  //EXPECT_EQ("qux", res[0]->lookupString("name"));
  //EXPECT_DOUBLE_EQ(53, res[0]->lookupDouble("x"));
  
  // Set up query: position.x < 40 and position.y > 0.  Reverse order
  // by the "name" metadata field.  Also, here we pull the message itself, not
  // just the metadata.  Finally, we can't use the simplified construction
  // syntax here because it's too long
/*  mongo_ros::Query q = mongo_ros::Query().append("x", mongo_ros::LT, 40).append("y", mongo_ros::GT, 0);
  vector<PoseMetaPtr> poses = coll.pullAllResults(q, false, "name", false);*/
  
  // Verify poses. 
  //EXPECT_EQ(3u, poses.size());
  //EXPECT_EQ(p1, *poses[0]);
  //EXPECT_EQ(p2, *poses[1]);
  //EXPECT_EQ(p1, *poses[2]);

  //EXPECT_EQ("oof", poses[0]->lookupString("name"));
  //EXPECT_EQ("baz", poses[1]->lookupString("name"));
  //EXPECT_EQ("bar", poses[2]->lookupString("name"));

  // Set up query to delete some poses.
  //mongo_ros::Query q2 ("y", mongo_ros::LT, 30);

  //EXPECT_EQ(5u, coll.count());
  //EXPECT_EQ(2u, coll.removeMessages(q2));
  //EXPECT_EQ(3u, coll.count());

  // Test findOne
  //mongo_ros::Query q4("name", "bar");
  //EXPECT_EQ(p1, *coll.findOne(q4, false));
  //EXPECT_DOUBLE_EQ(24, coll.findOne(q4, true)->lookupDouble("x"));

  //mongo_ros::Query q5("name", "barbar");
  //EXPECT_THROW(coll.findOne(q5, true), mongo_ros::NoMatchingMessageException);
  //EXPECT_THROW(coll.findOne(q5, false), mongo_ros::NoMatchingMessageException);
  
  // Test update
  //coll.modifyMetadata(q4, mongo_ros::Metadata("name", "barbar"));
  //EXPECT_EQ(3u, coll.count());
  //EXPECT_THROW(coll.findOne(q4, false), mongo_ros::NoMatchingMessageException);
  //ROS_INFO("here");
  //EXPECT_EQ(p1, *coll.findOne(q5, false));

  // Check stored metadata
  //boost::shared_ptr<mongo::DBClientConnection> conn = mongo_ros::makeDbConnection(ros::NodeHandle());
  //EXPECT_EQ("geometry_msgs/Pose", mongo_ros::messageType(*conn, "my_db", "poses"));
}


int main (int argc, char** argv)
{
  ros::init(argc, argv, "client_test");
  ros::NodeHandle nh;
  test();
}
