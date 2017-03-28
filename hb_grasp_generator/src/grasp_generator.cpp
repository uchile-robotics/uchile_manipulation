#include <ros/ros.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <eigen_stl_containers/eigen_stl_vector_container.h>
#include <iostream>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/Grasp.h>
#include <eigen_conversions/eigen_msg.h>

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

    class GraspPose
    {
    public:
        Eigen::Affine3d grasp;
        EigenSTL::vector_Affine3d pregrasp;
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    };

    class GraspGenerator
    {
      void generateGrasp(const Eigen::Affine3d& object_grasp_point, EigenSTL::vector_Affine3d& grasps);
    };

    class CylindricalGraspGeneratorOptions
    {
    public:
        // Yaw params
        int yaw_angle_count;
        // Pitch params
        int pitch_angle_count;
        double pitch_angle_min;
        double pitch_angle_max;
        double pitch_angle_res;
        // Roll params
        int roll_angle_count;
        double roll_angle_min;
        double roll_angle_max;
        double roll_angle_res;
        // Pregrasp params
        int pregrasp_count;
        double pregrasp_min;
        double pregrasp_max;
        double pregrasp_res;
        // Grasp point to end effector
        Eigen::Affine3d grasp_pose_to_eff;
        // Frames
        std::string base_link;
        std::string end_effector_name;
        std::string end_effector_parent_link;
        std::vector<std::string> joint_names;
        std::vector<double> grasp_posture;
        std::vector<double> pre_grasp_posture;
        trajectory_msgs::JointTrajectory grasp_posture_msg;
        trajectory_msgs::JointTrajectory pre_grasp_posture_msg;
        double pregrasp_time_from_start;
        double grasp_time_from_start;

    public:
        CylindricalGraspGeneratorOptions():
                // Default values
                yaw_angle_count(50),
                pitch_angle_count(5),
                pitch_angle_min(-M_PI/180.0 * 15.0), // 15 degrees
                pitch_angle_max( M_PI/180.0 * 15.0),
                pitch_angle_res((pitch_angle_max - pitch_angle_min)/pitch_angle_count),
                roll_angle_count(5),
                roll_angle_min(-M_PI/180.0 * 5.0), // 5 degrees
                roll_angle_max( M_PI/180.0 * 5.0),
                roll_angle_res((roll_angle_max - roll_angle_min)/roll_angle_count),
                pregrasp_count(5),
                pregrasp_min(0.1),
                pregrasp_max(0.3),
                pregrasp_res((pregrasp_max - pregrasp_min)/pregrasp_count)
        {}

        bool load(const ros::NodeHandle& nh, const std::string& end_effector)
        {
          // Load base_link a param
          if (!nh.hasParam("base_link"))
          {
            ROS_ERROR_STREAM_NAMED("grasp_data_loader","Grasp configuration parameter \"base_link\" missing from parameter server. Searching in namespace: " << nh.getNamespace());
            return false;
          }
          nh.getParam("base_link", base_link);

          // End effector nodehandle
          ros::NodeHandle ee_nh(nh, end_effector);

          // Load a param "pregrasp_time_from_start"
          if (!ee_nh.hasParam("pregrasp_time_from_start"))
          {
            ROS_ERROR_STREAM_NAMED("grasp_data_loader","Grasp configuration parameter \"pregrasp_time_from_start\" missing from parameter server. Searching in namespace: " << ee_nh.getNamespace());
            return false;
          }
          ee_nh.getParam("pregrasp_time_from_start", pregrasp_time_from_start);

          // Load a param "grasp_time_from_start"
          if (!ee_nh.hasParam("grasp_time_from_start"))
          {
            ROS_ERROR_STREAM_NAMED("grasp_data_loader","Grasp configuration parameter \"grasp_time_from_start\" missing from parameter server. Searching in namespace: " << ee_nh.getNamespace());
            return false;
          }
          ee_nh.getParam("grasp_time_from_start", grasp_time_from_start);

          // Load a param "end_effector_name"
          if (!ee_nh.hasParam("end_effector_name"))
          {
            ROS_ERROR_STREAM_NAMED("grasp_data_loader","Grasp configuration parameter \"end_effector_name\" missing from parameter server. Searching in namespace: " << ee_nh.getNamespace());
            return false;
          }
          ee_nh.getParam("end_effector_name", end_effector_name);

          // Load a param "end_effector_parent_link"
          if (!ee_nh.hasParam("end_effector_parent_link"))
          {
            ROS_ERROR_STREAM_NAMED("grasp_data_loader","Grasp configuration parameter \"end_effector_parent_link\" missing from parameter server. Searching in namespace: " << ee_nh.getNamespace());
            return false;
          }
          ee_nh.getParam("end_effector_parent_link", end_effector_parent_link);

          // Check for joints params
          if (!ee_nh.hasParam("joints"))
          {
            ROS_ERROR_STREAM_NAMED("grasp_data_loader","Grasp configuration parameter \"joints\" missing from parameter server. Searching in namespace: " << ee_nh.getNamespace());
            return false;
          }
          // Get joint names
          ee_nh.getParam("joints", joint_names);

          // Check for grasp_posture
          if(!ee_nh.hasParam("grasp_posture"))
          {
            ROS_ERROR_STREAM_NAMED("grasp_data_loader","Grasp configuration parameter \"grasp_posture\" missing from parameter server. Searching in namespace: " << ee_nh.getNamespace());
            return false;
          }
          // Get grasp_posture
          ee_nh.getParam("grasp_posture", grasp_posture);
          // Check grasp_posture lenght
          if (joint_names.size() != grasp_posture.size())
          {
            ROS_ERROR_STREAM_NAMED("grasp_data_loader","Parameter length of \"grasp_posture\" must be the same that \"joint_names\".");
            return false;
          }

          // Check for pregrasp_posture
          if(!ee_nh.hasParam("pregrasp_posture"))
          {
            ROS_ERROR_STREAM_NAMED("grasp_data_loader","Grasp configuration parameter \"pregrasp_posture\" missing from parameter server. Searching in namespace: " << ee_nh.getNamespace());
            return false;
          }
          // Get pregrasp_posture
          ee_nh.getParam("pregrasp_posture", pre_grasp_posture);
          // Check pregrasp_posture lenght
          if (joint_names.size() != pre_grasp_posture.size())
          {
            ROS_ERROR_STREAM_NAMED("grasp_data_loader","Parameter length of \"pregrasp_posture\" must be the same that \"joint_names\".");
            return false;
          }

          // --------------------------------------------------------
          // Grasp pose to end effector
          std::vector<double> pose_elements;
          if (!ee_nh.hasParam("grasp_pose_to_eef"))
          {
            ROS_WARN("Parameter \"grasp_pose_to_eef\" missing. Using default.");
            pose_elements.resize(6, 0.0); // Empty transform
          }
          ee_nh.getParam("grasp_pose_to_eef", pose_elements);
          // Check size
          if (pose_elements.size() != 6)
          {
            ROS_ERROR("Parameter \"grasp_pose_to_eef\" must have 6 components.");
            return false;
          }
          // Fill transform
          grasp_pose_to_eff = Eigen::Affine3d::Identity() * Eigen::AngleAxisd(pose_elements[3], Eigen::Vector3d::UnitX())
                               * Eigen::AngleAxisd(pose_elements[4], Eigen::Vector3d::UnitY())
                               * Eigen::AngleAxisd(pose_elements[5], Eigen::Vector3d::UnitZ());
          grasp_pose_to_eff.translation().x() = pose_elements[0];
          grasp_pose_to_eff.translation().y() = pose_elements[1];
          grasp_pose_to_eff.translation().z() = pose_elements[2];

          // -------------------------------
          // Create pre grasp posture
          pre_grasp_posture_msg.header.frame_id = base_link;
          pre_grasp_posture_msg.header.stamp = ros::Time::now();
          // Name of joints:
          pre_grasp_posture_msg.joint_names = joint_names;
          // Position of joints
          pre_grasp_posture_msg.points.resize(1);
          pre_grasp_posture_msg.points[0].positions = pre_grasp_posture;
          pre_grasp_posture_msg.points[0].time_from_start = ros::Duration(pregrasp_time_from_start);
          // -------------------------------
          // Create grasp posture
          grasp_posture_msg.header.frame_id = base_link;
          grasp_posture_msg.header.stamp = ros::Time::now();
          // Name of joints:
          grasp_posture_msg.joint_names = joint_names;
          // Position of joints
          grasp_posture_msg.points.resize(1);
          grasp_posture_msg.points[0].positions = grasp_posture;
          grasp_posture_msg.points[0].time_from_start = ros::Duration(grasp_time_from_start);

          // --------------------------------------------------------
          // Cylindrical grasp nodehandle
          ros::NodeHandle grasp_gen_nh(ee_nh, "cylindrical_grasp_generator");
          // --------------------------------------------------------
          // Grasp data
          // Yaw angle: 50 points
          grasp_gen_nh.param<int>("yaw_angle_count", yaw_angle_count, 50);

          // Pitch angle
          // 5 points
          grasp_gen_nh.param<int>("pitch_angle_count", pitch_angle_count, 5);
          // 15 degrees
          grasp_gen_nh.param<double>("pitch_angle_min", pitch_angle_min, -M_PI/180.0 * 15.0);
          grasp_gen_nh.param<double>("pitch_angle_max", pitch_angle_max,  M_PI/180.0 * 15.0);
          pitch_angle_res = (pitch_angle_max - pitch_angle_min)/pitch_angle_count;

          // Roll angle
          // 5 points
          grasp_gen_nh.param<int>("roll_angle_count", roll_angle_count, 5);
          // 5 degrees
          grasp_gen_nh.param<double>("roll_angle_min", roll_angle_min, -M_PI/180.0 * 5.0);
          grasp_gen_nh.param<double>("roll_angle_max", roll_angle_max,  M_PI/180.0 * 5.0);
          roll_angle_res = (roll_angle_max - roll_angle_min)/roll_angle_count;

          // --------------------------------------------------------
          // Pregrasp data
          // 3 points
          grasp_gen_nh.param<int>("pregrasp_count", pregrasp_count, 5);
          // 5 degrees
          grasp_gen_nh.param<double>("pregrasp_min", pregrasp_min, 0.1);
          grasp_gen_nh.param<double>("pregrasp_max", pregrasp_max, 0.3);
          pregrasp_res = (pregrasp_max - pregrasp_min)/pregrasp_count;
        }

        friend std::ostream& operator<<(std::ostream& os, const CylindricalGraspGeneratorOptions& opt);
    }; // CylindricalGraspGeneratorOptions

    std::ostream& operator<<(std::ostream& os, const CylindricalGraspGeneratorOptions& opt)
    {
      os << "Base link: " << opt.base_link << std::endl;
      os << "End effector: " << opt.end_effector_name << std::endl;
      os << "End effector parent: " << opt.end_effector_parent_link << std::endl;

      os << "Joint names: ";
      for(std::size_t i=0; i < opt.joint_names.size()-1; ++i) os << opt.joint_names[i] << ", ";
      os << opt.joint_names[opt.joint_names.size()-1] << std::endl;

      os << "Grasp posture: ";
      for(std::size_t i=0; i < opt.grasp_posture.size()-1; ++i) os << opt.grasp_posture[i] << ", ";
      os << opt.grasp_posture[opt.grasp_posture.size()-1] << std::endl;

      os << "Pregrasp posture: ";
      for(std::size_t i=0; i < opt.pre_grasp_posture.size()-1; ++i) os << opt.pre_grasp_posture[i] << ", ";
      os << opt.pre_grasp_posture[opt.pre_grasp_posture.size()-1] << std::endl;

      os << "Cylindrical grasp generator options:" << std::endl;
      os << "Yaw angle: count: " << opt.yaw_angle_count << std::endl;
      os << "Pitch angle: count: " << opt.pitch_angle_count << " (" << opt.pitch_angle_min
         << ", " << opt.pitch_angle_min << ")" << std::endl;
      os << "Roll angle: count: " << opt.roll_angle_count << " (" << opt.roll_angle_min
         << ", " << opt.roll_angle_max << ")" << std::endl;
      os << "Pregrasp options:" << std::endl;
      os << "Pregrasp count: " << opt.pregrasp_count << " (" << opt.pregrasp_min << ", "
         << opt.pregrasp_max << ")";
      return os;
    }

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

        void generateGrasp(const Eigen::Affine3d& object_grasp_point, std::vector<GraspPose>& grasps)
        {
          GraspPose grasp_pose;
          grasps.reserve(opt_.yaw_angle_count * opt_.roll_angle_count * opt_.pitch_angle_count);

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

                grasp_pose.grasp = object_grasp_point * yaw_rotation * pitch_rotation * roll_rotation;
                // Add pregrasp postions
                generatePregrasp(grasp_pose);
                // Add grasp point
                grasps.push_back(grasp_pose);

                // Publish poses
                if (verbose_)
                {
                  visual_tools_->publishAxis(grasp_pose.grasp);
                  visual_tools_->triggerBatchPublish();
                }
              }
            }
          }
        }

        bool generateAxisGrasps(const geometry_msgs::Pose& object_pose, std::vector<moveit_msgs::Grasp>& possible_grasps)
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

        void generatePregrasp(GraspPose& grasp_pose)
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
        }

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
  Eigen::Affine3d object_center = Eigen::Affine3d::Identity() * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX())
                                  * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY())
                                  * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ());
  object_center.translation().x() = 1.0; // Move one meter on x axis
  // Generate grasp positions
  std::vector<hb_grasp_generator::GraspPose> grasps;
  ros::Time t0 = ros::Time::now();
  demo.generateGrasp(object_center, grasps);
  ROS_INFO_STREAM("Generated grasps: " << grasps.size() << " sec: " << (ros::Time::now() - t0).toSec());
  return 0;
}