#ifndef HB_GRASP_GRASP_GENERATOR_H
#define HB_GRASP_GRASP_GENERATOR_H

#include <ros/ros.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/Grasp.h>

#include "hb_grasp_generator/grasp_options.h"

namespace hb_grasp_generator
{
geometry_msgs::PoseStamped getPreGraspPose(const moveit_msgs::Grasp &grasp, const std::string &ee_parent_link);

class GraspGenerator
{
 public:
  virtual bool generateGrasp(const geometry_msgs::Pose &object_pose,
                             std::vector<moveit_msgs::Grasp> &possible_grasps) = 0;
};

class CylindricalGraspGenerator : public GraspGenerator
{
 private:
  // Class name
  const std::string name_;
  // A shared node handle
  ros::NodeHandle nh_;
  // Global options
  GraspOptions global_opt_;
  // Grasp generator options
  CylindricalGraspGeneratorOptions opt_;

 public:
  /**
   * \brief Constructor
   */
  CylindricalGraspGenerator(const ros::NodeHandle &nh, const hb_grasp_generator::GraspOptions &opt);
  bool generateGrasp(const geometry_msgs::Pose &object_pose, std::vector<moveit_msgs::Grasp> &possible_grasps);

  /**
   * \brief Destructor
   */
  ~CylindricalGraspGenerator()
  {};
};  // end class
typedef boost::shared_ptr<CylindricalGraspGenerator> CylindricalGraspGeneratorPtr;

}  // namespace hb_grasp_generator

#endif //HB_GRASP_GRASP_GENERATOR_H
