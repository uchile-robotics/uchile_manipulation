#include <ros/ros.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <eigen_stl_containers/eigen_stl_vector_container.h>
#include <iostream>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

namespace hb_grasp_generator
{
    class GraspFilter
    {
    public:
        GraspFilter(robot_state::RobotState robot_state, moveit_visual_tools::MoveItVisualToolsPtr& visual_tools):
                robot_state_(robot_state),
                visual_tools_(visual_tools)
        {

        }
    private:
        // State of robot
        robot_state::RobotState robot_state_;

        // threaded kinematic solvers
        std::map<std::string, std::vector<kinematics::KinematicsBaseConstPtr> > kin_solvers_;

        // class for publishing stuff to rviz
        moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;
    };



    class GraspGenerator
    {
      void generateGrasp(const Eigen::Affine3d& object_grasp_point, EigenSTL::vector_Affine3d& grasps);
      void generatePregrasp(const EigenSTL::vector_Affine3d& grasps, const EigenSTL::vector_Affine3d& pregrasps);
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

        bool load(const ros::NodeHandle& nh)
        {
          // Child nodehandle
          ros::NodeHandle child_nh(nh, "cylindrical_grasp_generator");

          // --------------------------------------------------------
          // Grasp data

          // Yaw angle: 50 points
          child_nh.param<int>("yaw_angle_count", yaw_angle_count, 50);

          // Pitch angle
          // 5 points
          child_nh.param<int>("pitch_angle_count", pitch_angle_count, 5);
          // 15 degrees
          child_nh.param<double>("pitch_angle_min", pitch_angle_min, -M_PI/180.0 * 15.0);
          child_nh.param<double>("pitch_angle_max", pitch_angle_max,  M_PI/180.0 * 15.0);
          pitch_angle_res = (pitch_angle_max - pitch_angle_min)/pitch_angle_count;

          // Roll angle
          // 5 points
          child_nh.param<int>("roll_angle_count", roll_angle_count, 5);
          // 5 degrees
          child_nh.param<double>("roll_angle_min", roll_angle_min, -M_PI/180.0 * 5.0);
          child_nh.param<double>("roll_angle_max", roll_angle_max,  M_PI/180.0 * 5.0);
          roll_angle_res = (roll_angle_max - roll_angle_min)/roll_angle_count;

          // --------------------------------------------------------
          // Pregrasp data
          // 3 points
          child_nh.param<int>("pregrasp_count", pregrasp_count, 5);
          // 5 degrees
          child_nh.param<double>("pregrasp_min", pregrasp_min, 0.1);
          child_nh.param<double>("pregrasp_max", pregrasp_max, 0.3);
          pregrasp_res = (pregrasp_max - pregrasp_min)/pregrasp_count;
        }

        friend std::ostream& operator<<(std::ostream& os, const CylindricalGraspGeneratorOptions& opt);
    }; // CylindricalGraspGeneratorOptions

    std::ostream& operator<<(std::ostream& os, const CylindricalGraspGeneratorOptions& opt)
    {
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
          opt_.load(nh_);
          ROS_INFO_STREAM_NAMED(name_, opt_);

          // Clear messages
          visual_tools_->deleteAllMarkers();
          visual_tools_->enableBatchPublishing();
        }

        void generateGrasp(const Eigen::Affine3d& object_grasp_point, EigenSTL::vector_Affine3d& grasps)
        {
          Eigen::Affine3d grasp_pose;

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

                grasp_pose = object_grasp_point * yaw_rotation * pitch_rotation * roll_rotation;
                // Add grasp point
                grasps.push_back(grasp_pose);

                // Publish poses
                if (verbose_)
                {
                  visual_tools_->publishAxis(grasp_pose);
                  visual_tools_->triggerBatchPublish();
                }
              }
            }
          }
        }

        void generatePregrasp(const EigenSTL::vector_Affine3d& grasps, EigenSTL::vector_Affine3d& pregrasps)
        {
          Eigen::Affine3d pregrasp_pose;
          // Iterate over grasps positions
          EigenSTL::vector_Affine3d::const_iterator grasp_pose;
          for(grasp_pose = grasps.begin(); grasp_pose != grasps.end(); ++grasp_pose)
          {
            Eigen::Affine3d pregrasp_traslation = Eigen::Affine3d::Identity();

            for(std::size_t pregrasp_idx = 0; pregrasp_idx < opt_.pregrasp_count; ++pregrasp_idx)
            {
              // Generate translation on x axis
              pregrasp_traslation.translation().x() = opt_.pregrasp_min + pregrasp_idx*opt_.pregrasp_res;
              pregrasp_pose = *grasp_pose * pregrasp_traslation;
              pregrasps.push_back(pregrasp_pose);
              // Publish poses
              if (verbose_)
              {
                visual_tools_->publishAxis(pregrasp_pose);
                visual_tools_->triggerBatchPublish();
              }
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
  EigenSTL::vector_Affine3d grasp;
  ros::Time t0 = ros::Time::now();
  demo.generateGrasp(object_center, grasp);
  ROS_INFO_STREAM("Generated grasps: " << grasp.size() << " sec: " << (ros::Time::now() - t0).toSec());
  // Generate pregrasp positions
  EigenSTL::vector_Affine3d pregrasp;
  t0 = ros::Time::now();
  demo.generatePregrasp(grasp, pregrasp);
  ROS_INFO_STREAM("Generated pregrasps: " << pregrasp.size() << " sec: " << (ros::Time::now() - t0).toSec());

  return 0;
}