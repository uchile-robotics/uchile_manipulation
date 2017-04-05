#include <iostream>
// Eigen
#include <eigen_stl_containers/eigen_stl_vector_container.h>
#include <eigen_conversions/eigen_msg.h>

#include "hb_grasp_generator/grasp_options.h"
#include "hb_grasp_generator/grasp_generator.h"

namespace hb_grasp_generator
{
/**
 * Get the pregrasp pose from a grasp message.
 * @param grasp Grasp message.
 * @param ee_parent_link
 * @return
 */
geometry_msgs::PoseStamped getPreGraspPose(const moveit_msgs::Grasp &grasp, const std::string &ee_parent_link)
{
  // Grasp Pose Variables
  geometry_msgs::PoseStamped grasp_pose = grasp.grasp_pose;
  Eigen::Affine3d grasp_pose_eigen;
  tf::poseMsgToEigen(grasp_pose.pose, grasp_pose_eigen);

  // Get pre-grasp pose first
  geometry_msgs::PoseStamped pre_grasp_pose;
  Eigen::Affine3d pre_grasp_pose_eigen = grasp_pose_eigen; // Copy original grasp pose to pre-grasp pose

  // Approach direction variables
  Eigen::Vector3d pre_grasp_approach_direction_local;

  // The direction of the pre-grasp
  // Calculate the current animation position based on the percent
  Eigen::Vector3d pre_grasp_approach_direction = Eigen::Vector3d(
      -1 * grasp.pre_grasp_approach.direction.vector.x * grasp.pre_grasp_approach.desired_distance,
      -1 * grasp.pre_grasp_approach.direction.vector.y * grasp.pre_grasp_approach.desired_distance,
      -1 * grasp.pre_grasp_approach.direction.vector.z * grasp.pre_grasp_approach.desired_distance
  );

  // Decide if we need to change the approach_direction to the local frame of the end effector orientation
  if (grasp.pre_grasp_approach.direction.header.frame_id == ee_parent_link)
  {
    // Apply/compute the approach_direction vector in the local frame of the grasp_pose orientation
    pre_grasp_approach_direction_local = grasp_pose_eigen.rotation() * pre_grasp_approach_direction;
  } else
  {
    pre_grasp_approach_direction_local =
        pre_grasp_approach_direction; //grasp_pose_eigen.rotation() * pre_grasp_approach_direction;
  }

  // Update the grasp matrix usign the new locally-framed approach_direction
  pre_grasp_pose_eigen.translation() += pre_grasp_approach_direction_local;

  // Convert eigen pre-grasp position back to regular message
  tf::poseEigenToMsg(pre_grasp_pose_eigen, pre_grasp_pose.pose);

  // Copy original header to new grasp
  pre_grasp_pose.header = grasp_pose.header;

  return pre_grasp_pose;
}

CylindricalGraspGenerator::CylindricalGraspGenerator() :
    nh_("~"),
    opt_(),
    name_("cylindrical_grasp_generator")
{
  opt_.load(nh_, "l_gripper");
  ROS_DEBUG_STREAM_NAMED(name_, opt_);
}

bool CylindricalGraspGenerator::generateGrasp(const geometry_msgs::Pose &object_pose,
                                              std::vector<moveit_msgs::Grasp> &possible_grasps)
{
  // ------------------------------------------------------------------------------------------
  // Create a transform from the object's frame (center of object) to /base_link
  Eigen::Affine3d object_global_transform;
  tf::poseMsgToEigen(object_pose, object_global_transform);

  // ------------------------------------------------------------------------------------------
  // Approach
  // Create approach motion
  moveit_msgs::GripperTranslation pre_grasp_approach;
  pre_grasp_approach.direction.header.stamp = ros::Time::now();

  // ------------------------------------------------------------------------------------------
  // Retreat
  // Create retreat motion
  moveit_msgs::GripperTranslation post_grasp_retreat;
  post_grasp_retreat.direction.header.stamp = ros::Time::now();

  // ---------------------------------------------------------------------------------------------
  // Grasp generator loop

  // Grasp id
  static int grasp_id = 0;

  for (std::size_t yaw_angle_idx = 0; yaw_angle_idx < opt_.yaw_angle_count; ++yaw_angle_idx)
  {
    // Yaw angle generation (rotation on Z axis)
    Eigen::Affine3d yaw_rotation = Eigen::Affine3d::Identity() * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX())
        * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(2 * M_PI * yaw_angle_idx / opt_.yaw_angle_count, Eigen::Vector3d::UnitZ());

    for (std::size_t pitch_angle_idx = 0; pitch_angle_idx < opt_.pitch_angle_count; ++pitch_angle_idx)
    {
      // Pitch angle generation (rotation on Y axis)
      double pitch_angle = opt_.pitch_angle_min + pitch_angle_idx * opt_.pitch_angle_res;
      Eigen::Affine3d pitch_rotation = Eigen::Affine3d::Identity() * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX())
          * Eigen::AngleAxisd(pitch_angle, Eigen::Vector3d::UnitY())
          * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ());

      for (std::size_t roll_angle_idx = 0; roll_angle_idx < opt_.roll_angle_count; ++roll_angle_idx)
      {
        // Roll angle generation (rotation on X axis)
        double roll_angle = opt_.roll_angle_min + roll_angle_idx * opt_.roll_angle_res;
        Eigen::Affine3d
            roll_rotation = Eigen::Affine3d::Identity() * Eigen::AngleAxisd(roll_angle, Eigen::Vector3d::UnitX())
            * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY())
            * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ());
        // Create grasp pose
        Eigen::Affine3d grasp_pose = object_global_transform * yaw_rotation * pitch_rotation * roll_rotation;
        // Create a Grasp message
        moveit_msgs::Grasp new_grasp;
        // Assign grasp id
        new_grasp.id = "grasp" + boost::lexical_cast<std::string>(grasp_id);
        ++grasp_id;

        // ------------------------------------------------------------------------
        // Pregrasp and grasp postures
        // The internal posture of the hand for the pre-grasp only positions are used
        new_grasp.pre_grasp_posture = opt_.pre_grasp_posture_msg;
        // The internal posture of the hand for the grasp positions and efforts are used
        new_grasp.grasp_posture = opt_.grasp_posture_msg;

        // Change grasp to frame of reference of this custom end effector
        grasp_pose = grasp_pose * opt_.grasp_pose_to_eff;

        // Fill grasp message with grasp pose and header info
        tf::poseEigenToMsg(grasp_pose, new_grasp.grasp_pose.pose);
        new_grasp.grasp_pose.header.stamp = ros::Time::now();
        new_grasp.grasp_pose.header.frame_id = opt_.base_link;

        // The maximum contact force to use while grasping (<=0 to disable)
        new_grasp.max_contact_force = 0;

        // ------------------------------------------------------------------------
        // Approach vector
        // Turn around y axis and apply end effector transform
        Eigen::Vector3d approach_vector =
            -1.0 * (Eigen::AngleAxisd(opt_.approach_angle, Eigen::Vector3d::UnitY()) * Eigen::Vector3d::UnitX());
        approach_vector = opt_.grasp_pose_to_eff * approach_vector;

        // ------------------------------------------------------------------------
        // Approach
        tf::vectorEigenToMsg(approach_vector, pre_grasp_approach.direction.vector);
        pre_grasp_approach.direction.header.frame_id = opt_.end_effector_parent_link;
        new_grasp.pre_grasp_approach = pre_grasp_approach;

        // ------------------------------------------------------------------------
        // Retreat
        tf::vectorEigenToMsg(-1.0 * approach_vector, post_grasp_retreat.direction.vector);
        post_grasp_retreat.direction.header.frame_id = opt_.end_effector_parent_link;
        new_grasp.post_grasp_retreat = post_grasp_retreat;

        // ------------------------------------------------------------------------
        // Approach and retreat loop
        for (std::size_t distance_idx = 0; distance_idx < opt_.pregrasp_count; ++distance_idx)
        {
          float distance = opt_.pregrasp_min + opt_.pregrasp_res * distance_idx;
          // The distance the origin of a robot link needs to travel
          pre_grasp_approach.desired_distance = distance;
          pre_grasp_approach.min_distance = 0.9 * distance; // TODO Set as param

          post_grasp_retreat.desired_distance = distance;
          post_grasp_retreat.min_distance = 0.9 * distance; // TODO Set as param

          // Add to vector
          possible_grasps.push_back(new_grasp);
        }
      }
    }
  }
  ROS_DEBUG_STREAM_NAMED("grasp", "Generated " << possible_grasps.size() << " grasps.");
  return true;
}
}  // namespace hb_grasp_generator
