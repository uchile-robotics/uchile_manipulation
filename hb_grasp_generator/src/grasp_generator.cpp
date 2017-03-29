#include <ros/ros.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <eigen_stl_containers/eigen_stl_vector_container.h>
#include <iostream>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/Grasp.h>
#include <eigen_conversions/eigen_msg.h>

#include "hb_grasp_generator/grasp_options.h"

namespace hb_grasp_generator
{
    class GraspFilter
    {
    public:
        GraspFilter(robot_state::RobotState robot_state, moveit_visual_tools::MoveItVisualToolsPtr& visual_tools):
                robot_state_(robot_state),
                visual_tools_(visual_tools)
        {}
    private:
        // State of robot
        robot_state::RobotState robot_state_;

        // Threaded kinematic solvers
        std::map<std::string, std::vector<kinematics::KinematicsBaseConstPtr> > kin_solvers_;

        // Visualization on RViz
        moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;
    };

    class GraspGenerator
    {
        bool generateGrasp(const geometry_msgs::Pose& object_pose, std::vector<moveit_msgs::Grasp>& possible_grasps);
    };

    class CylindricalGraspGenerator : public GraspGenerator
    {
    private:
        // A shared node handle
        ros::NodeHandle nh_;
        // Grasp generator options
        CylindricalGraspGeneratorOptions opt_;
        // For visualizing things in rviz
        rviz_visual_tools::RvizVisualToolsPtr visual_tools_;

        std::string name_;
        // Verbose
        bool verbose_;

    public:
        /**
         * \brief Constructor
         */
        CylindricalGraspGenerator():
                nh_("~"),
                opt_(),
                name_("cylindrical_grasp_generator"),
                verbose_(true)
        {
          visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("base_link", "/grasp"));
          ros::Duration(2.0).sleep();
          opt_.load(nh_, "l_gripper");
          ROS_INFO_STREAM_NAMED(name_, opt_);

          // Clear messages
          visual_tools_->deleteAllMarkers();
          visual_tools_->enableBatchPublishing();
        }

        bool generateGrasp(const geometry_msgs::Pose& object_pose, std::vector<moveit_msgs::Grasp>& possible_grasps)
        {
          Eigen::Affine3d object_global_transform;
          // ------------------------------------------------------------------------------------------
          // Create a transform from the object's frame (center of object) to /base_link
          tf::poseMsgToEigen(object_pose, object_global_transform);

          // ------------------------------------------------------------------------------------------
          // Approach
          // Create approach motion
          moveit_msgs::GripperTranslation pre_grasp_approach;
          pre_grasp_approach.direction.header.stamp = ros::Time::now();
          // The distance the origin of a robot link needs to travel
          pre_grasp_approach.desired_distance = opt_.pregrasp_max;
          pre_grasp_approach.min_distance = opt_.pregrasp_min;

          // ------------------------------------------------------------------------------------------
          // Retreat
          // Create retreat motion
          moveit_msgs::GripperTranslation post_grasp_retreat;
          post_grasp_retreat.direction.header.stamp = ros::Time::now();
          // The distance the origin of a robot link needs to travel
          post_grasp_retreat.desired_distance = opt_.pregrasp_max;
          post_grasp_retreat.min_distance =  opt_.pregrasp_min;

          // Create re-usable blank pose
          geometry_msgs::PoseStamped grasp_pose_msg;
          grasp_pose_msg.header.stamp = ros::Time::now();
          grasp_pose_msg.header.frame_id = opt_.base_link;

          // ---------------------------------------------------------------------------------------------
          // Grasp generator loop

          // Grasp id
          static int grasp_id = 0;

          for (std::size_t yaw_angle_idx = 0; yaw_angle_idx < opt_.yaw_angle_count; ++yaw_angle_idx)
          {
            // Yaw angle generation (rotation on Z axis)
            Eigen::Affine3d yaw_rotation = Eigen::Affine3d::Identity() * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX())
                                           * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY())
                                           * Eigen::AngleAxisd(2*M_PI*yaw_angle_idx/opt_.yaw_angle_count, Eigen::Vector3d::UnitZ());

            for (std::size_t pitch_angle_idx = 0; pitch_angle_idx < opt_.pitch_angle_count; ++pitch_angle_idx)
            {
              // Pitch angle generation (rotation on Y axis)
              double pitch_angle = opt_.pitch_angle_min + pitch_angle_idx*opt_.pitch_angle_res;
              Eigen::Affine3d pitch_rotation = Eigen::Affine3d::Identity() * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX())
                                               * Eigen::AngleAxisd(pitch_angle, Eigen::Vector3d::UnitY())
                                               * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ());

              for (std::size_t roll_angle_idx = 0; roll_angle_idx < opt_.roll_angle_count; ++roll_angle_idx)
              {
                // Roll angle generation (rotation on X axis)
                double roll_angle = opt_.roll_angle_min + roll_angle_idx*opt_.roll_angle_res;
                Eigen::Affine3d roll_rotation = Eigen::Affine3d::Identity() * Eigen::AngleAxisd(roll_angle, Eigen::Vector3d::UnitX())
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

                // Fill grasp message with grasp pose
                tf::poseEigenToMsg(grasp_pose, new_grasp.grasp_pose.pose);

                // The maximum contact force to use while grasping (<=0 to disable)
                new_grasp.max_contact_force = 0;

                // ------------------------------------------------------------------------
                // Approach vector
                double approach_angle = 0.0; // TODO: Parametro
                Eigen::Vector3d approach_vector = -1*(Eigen::AngleAxisd(approach_angle, Eigen::Vector3d::UnitY()) * Eigen::Vector3d::UnitX());
                approach_vector = opt_.grasp_pose_to_eff * approach_vector;

                // ------------------------------------------------------------------------
                // Approach
                tf::vectorEigenToMsg(approach_vector, pre_grasp_approach.direction.vector);
                pre_grasp_approach.direction.header.frame_id = opt_.end_effector_parent_link;
                new_grasp.pre_grasp_approach = pre_grasp_approach;

                // ------------------------------------------------------------------------
                // Retreat
                tf::vectorEigenToMsg(-1*approach_vector, post_grasp_retreat.direction.vector);
                post_grasp_retreat.direction.header.frame_id = opt_.end_effector_parent_link;
                new_grasp.post_grasp_retreat = post_grasp_retreat;

                // Add to vector
                possible_grasps.push_back(new_grasp);

                // Publish poses
                if (verbose_)
                {
                  visual_tools_->publishAxis(grasp_pose);
                  visual_tools_->triggerBatchPublish();
                }
              }
            }
          }
          ROS_DEBUG_STREAM_NAMED("grasp", "Generated " << possible_grasps.size() << " grasps." );
          return true;
        }

/*        void generatePregrasp(GraspPose& grasp_pose)
        {
          Eigen::Affine3d pregrasp_pose;
          Eigen::Affine3d pregrasp_traslation = Eigen::Affine3d::Identity();
          grasp_pose.pregrasp.reserve(opt_.pregrasp_count);
          for(std::size_t pregrasp_idx = 0; pregrasp_idx < opt_.pregrasp_count; ++pregrasp_idx)
          {
            // Generate translation on x axis
            pregrasp_traslation.translation().x() = opt_.pregrasp_min + pregrasp_idx*opt_.pregrasp_res;
            pregrasp_pose = grasp_pose.grasp * pregrasp_traslation * opt_.grasp_pose_to_eff;

            grasp_pose.pregrasp.push_back(pregrasp_pose);
            // Publish poses
            if (verbose_)
            {
              visual_tools_->publishAxis(pregrasp_pose);
              visual_tools_->triggerBatchPublish();
            }
          }
        }*/

        /**
         * \brief Destructor
         */
        ~CylindricalGraspGenerator() {}
    };  // end class

}  // namespace hb_grasp_generator

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cylindrical_grasp_generator_test");
  ROS_INFO_STREAM("Grasp generator Demo");

  // Allow the action server to recieve and send ros messages
  ros::AsyncSpinner spinner(1);
  spinner.start();

  hb_grasp_generator::CylindricalGraspGenerator demo;

  // Create object pose
  geometry_msgs::Pose pose_msg;
  Eigen::Affine3d pose = Eigen::Affine3d::Identity() * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX())
                                  * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY())
                                  * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ());
  pose.translation().x() = 1.0; // Move one meter on x axis
  tf::poseEigenToMsg(pose, pose_msg);
  // Generate grasp positions
  std::vector<moveit_msgs::Grasp> grasps;
  ros::Time t0 = ros::Time::now();
  demo.generateGrasp(pose_msg, grasps);
  ROS_INFO_STREAM("Generated grasps: " << grasps.size() << " sec: " << (ros::Time::now() - t0).toSec());
  return 0;
}