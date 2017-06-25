#ifndef HB_GRASP_GRASP_FILTER_H
#define HB_GRASP_GRASP_FILTER_H

// ROS
#include <ros/ros.h>
#include <tf_conversions/tf_eigen.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit_msgs/Grasp.h>

// MoveIt
#include <moveit/robot_state/robot_state.h>
#include <moveit/kinematics_plugin_loader/kinematics_plugin_loader.h>

// C++
#include <boost/thread.hpp>
#include <math.h>

namespace hb_grasp_generator
{

// Struct for passing parameters to threads, for cleaner code
struct IkThreadStruct
{
  IkThreadStruct(
      std::vector<moveit_msgs::Grasp> &possible_grasps, // the input
      std::vector<moveit_msgs::Grasp> &filtered_grasps, // the result
      std::vector<trajectory_msgs::JointTrajectoryPoint> &ik_solutions, // the resulting solutions for each filtered grasp
      Eigen::Affine3d &link_transform,
      int grasps_id_start,
      int grasps_id_end,
      kinematics::KinematicsBaseConstPtr kin_solver,
      bool filter_pregrasp,
      std::string ee_parent_link,
      double timeout,
      boost::mutex *lock,
      int thread_id)
      : possible_grasps_(possible_grasps),
        filtered_grasps_(filtered_grasps),
        ik_solutions_(ik_solutions),
        link_transform_(link_transform),
        grasps_id_start_(grasps_id_start),
        grasps_id_end_(grasps_id_end),
        kin_solver_(kin_solver),
        filter_pregrasp_(filter_pregrasp),
        ee_parent_link_(ee_parent_link),
        timeout_(timeout),
        lock_(lock),
        thread_id_(thread_id)
  {
  }
  std::vector<moveit_msgs::Grasp> &possible_grasps_;
  std::vector<moveit_msgs::Grasp> &filtered_grasps_;
  std::vector<trajectory_msgs::JointTrajectoryPoint> &ik_solutions_;
  Eigen::Affine3d link_transform_;
  int grasps_id_start_;
  int grasps_id_end_;
  kinematics::KinematicsBaseConstPtr kin_solver_;
  bool filter_pregrasp_;
  std::string ee_parent_link_;
  double timeout_;
  boost::mutex *lock_;
  int thread_id_;
};

// Class
class GraspFilter
{
 private:
  // State of robot
  robot_state::RobotState robot_state_;
  // Kinematic solvers
  std::map<std::string, std::vector<kinematics::KinematicsBaseConstPtr> > kin_solvers_;
  // Verbose mode
  bool verbose_;
  // Log namespace
  const std::string name_;

 public:

  // Constructor
  GraspFilter(const robot_state::RobotState& robot_state);

  // Destructor
  ~GraspFilter();

  // Of an array of grasps, choose just one for use
  bool chooseBestGrasp(const std::vector<moveit_msgs::Grasp> &possible_grasps,
                       moveit_msgs::Grasp &chosen);

  // Take the nth grasp from the array
  bool filterNthGrasp(std::vector<moveit_msgs::Grasp> &possible_grasps, int n);

  /**
   * \brief Choose the 1st grasp that is kinematically feasible
   * \param
   * \param
   * \param whether to also check ik feasibility for the pregrasp position
   * \return true on success
   */
  /**
   * @brief Filter grasps kinematically .
   * @param possible_grasps Grasps from grasp generator function.
   * @param ik_solutions IK solutions found.
   * @param filter_pregrasp
   * @param ee_parent_link
   * @param planning_group
   * @return
   */
  bool filterGrasps(std::vector<moveit_msgs::Grasp> &possible_grasps,
                    std::vector<trajectory_msgs::JointTrajectoryPoint> &ik_solutions,
                    bool filter_pregrasp, const std::string &ee_parent_link,
                    const std::string &planning_group,
                    const double override_ik_timeout = 0.0);

 private:

  /**
   * \brief Thread for checking part of the possible grasps list
   * \param
   */
  void filterGraspThread(IkThreadStruct ik_thread_struct);

}; // end of class

typedef boost::shared_ptr<GraspFilter> GraspFilterPtr;
typedef boost::shared_ptr<const GraspFilter> GraspFilterConstPtr;

} // namespace

#endif //  HB_GRASP_GRASP_FILTER_H
