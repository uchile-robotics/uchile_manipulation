#include <ros/ros.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <eigen_stl_containers/eigen_stl_vector_container.h>

namespace hb_grasp_generator
{

    class GraspGenerator
    {
        void generateGrasp(const Eigen::Affine3d& object_grasp_point, EigenSTL::vector_Affine3d& grasps);
        void generatePregrasp(const EigenSTL::vector_Affine3d& grasps, const EigenSTL::vector_Affine3d& pregrasps);
    };

    class CylindricalGraspGenerator : public GraspGenerator
    {
    private:
        // A shared node handle
        ros::NodeHandle nh_;

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
                nh_(),
                name_("rviz_demo"),
                verbose_(true)
        {
          visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("base_link", "/grasp"));

          ROS_INFO("Sleeping 3 seconds before running demo");
          ros::Duration(2.0).sleep();

          // Clear messages
          visual_tools_->deleteAllMarkers();
          visual_tools_->enableBatchPublishing();
        }

        void generateGrasp(const Eigen::Affine3d& object_grasp_point, EigenSTL::vector_Affine3d& grasps)
        {
          Eigen::Affine3d grasp_pose;

          // Yaw params
          const std::size_t yaw_angle_count = 4;
          // Pitch params
          const std::size_t pitch_angle_count = 2;
          const double pitch_angle_min = - M_PI/180.0 * 15.0; // 15 degrees
          const double pitch_angle_max = M_PI/180.0 * 15.0;
          const double pitch_angle_res = (pitch_angle_max - pitch_angle_min)/pitch_angle_count;
          // Roll params
          const std::size_t roll_angle_count = 3;
          const double roll_angle_min = - M_PI/180.0 * 5.0; // 5 degrees
          const double roll_angle_max = M_PI/180.0 * 5.0;
          const double roll_angle_res = (pitch_angle_max - pitch_angle_min)/pitch_angle_count;

          // --------------------------------------------------------------------
          ROS_INFO_STREAM_NAMED(name_, "Object center");

          for (std::size_t yaw_angle_idx = 0; yaw_angle_idx < yaw_angle_count; ++yaw_angle_idx)
          {
            // Yaw angle generation (rotation on Z axis)
            Eigen::Affine3d yaw_rotation = Eigen::Affine3d::Identity() * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX())
                                             * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY())
                                             * Eigen::AngleAxisd(2*M_PI*yaw_angle_idx/yaw_angle_count, Eigen::Vector3d::UnitZ());

            for (std::size_t pitch_angle_idx = 0; pitch_angle_idx < pitch_angle_count; ++pitch_angle_idx)
            {
              // Pitch angle generation (rotation on Y axis)
              double pitch_angle = pitch_angle_min + pitch_angle_idx*pitch_angle_res;
              Eigen::Affine3d pitch_rotation = Eigen::Affine3d::Identity() * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX())
                                                   * Eigen::AngleAxisd(pitch_angle, Eigen::Vector3d::UnitY())
                                                   * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ());

              for (std::size_t roll_angle_idx = 0; roll_angle_idx < roll_angle_count; ++roll_angle_idx)
              {
                // Roll angle generation (rotation on X axis)
                double roll_angle = roll_angle_min + roll_angle_idx*roll_angle_res;
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
          // Pregrasp params
          const std::size_t pregrap_count = 3;
          const double pregrap_min = 0.1; // 10 cm
          const double pregrap_max = 0.2; // 20 cm
          const double pregrap_res = (pregrap_max - pregrap_min)/pregrap_count;
          // Iterate over grasps positions
          EigenSTL::vector_Affine3d::const_iterator grasp_pose;
          for(grasp_pose = grasps.begin(); grasp_pose != grasps.end(); ++grasp_pose)
          {
            Eigen::Affine3d pregrasp_traslation = Eigen::Affine3d::Identity();

            for(std::size_t pregrasp_idx = 0; pregrasp_idx < pregrap_count; ++pregrasp_idx)
            {
              // Generate translation on x axis
              pregrasp_traslation.translation().x() = pregrap_min + pregrasp_idx*pregrap_res;
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
  ros::init(argc, argv, "visual_tools_demo");
  ROS_INFO_STREAM("Visual Tools Demo");

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