#include <bender_arm_planning/capability_map.h>
#include <ros/ros.h>
#include <moveit_msgs/CollisionObject.h>
#include <shape_msgs/SolidPrimitive.h>
#include <geometry_msgs/Pose.h>


int main(int argc, char** argv) 
{
  using namespace capability_map;
  ros::init(argc, argv, "cm_load_test");
  ros::NodeHandle nh;

  /* --------------------------------------------------------------------------------
  * Capability Map
  */
  CapabilityMapPtr cm = loadCapMap("/home/rodrigo/.ros/store2.dat");
  // Mostrar
  ROS_INFO_STREAM("Capability Map\n" << *cm);

  double z_res = cm->getResolution('z');

  /* --------------------------------------------------------------------------------
  * Construir objeto
  */
  double target_height = 0.22;

  geometry_msgs::Pose p;
  p.position.x = 0.52;
  p.position.y = 0.22;
  p.position.z = 0.80;
  p.orientation.w = 1.0;

  moveit_msgs::CollisionObject obj;
  obj.header.stamp = ros::Time::now();
  obj.header.frame_id = "bender/base_link";
  obj.id = "pringles";

  shape_msgs::SolidPrimitive cylinder;
  cylinder.type = shape_msgs::SolidPrimitive::CYLINDER;
  cylinder.dimensions.push_back(target_height);
  cylinder.dimensions.push_back(0.04);

  obj.primitives.push_back(cylinder);
  obj.primitive_poses.push_back(p);

  obj.operation = moveit_msgs::CollisionObject::ADD;

  /* --------------------------------------------------------------------------------
  * Obtener grasps
  */
  int n = target_height/z_res;

  Eigen::Vector3d target(p.position.x, p.position.y, p.position.z - target_height/2);
  Eigen::Vector3d grasp_saved, error;
  
  std::vector<GraspPoint> grasp(n);
  
  for (int i = 0; i < n; ++i)
  {
    if (cm->getGrasp(target, grasp[i]))
    {
      ROS_WARN("GraspPoint added!");
      grasp[i].getPosition(grasp_saved); // Posicion guardada en Cap map
      error = grasp_saved - target;
      ROS_INFO_STREAM("Target " << target.transpose() 
        << " Saved " << grasp_saved.transpose() << " Error " << error.transpose());
    }
    ROS_INFO_STREAM("Target " << target.transpose());
    target(2) += z_res;
  }

  return 0;
}
