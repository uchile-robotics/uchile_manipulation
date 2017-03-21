#include <ros/ros.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <eigen_stl_containers/eigen_stl_vector_container.h>

namespace hb_grasp_generator
{

    class GraspGenerator
    {
        void generateGrasp(const Eigen::Affine3d& object_grasp_point, EigenSTL::vector_Affine3d& grasps);
    };

    class CylindricalGraspGenerator : public GraspGenerator
    {
    private:
        // A shared node handle
        ros::NodeHandle nh_;

        // For visualizing things in rviz
        rviz_visual_tools::RvizVisualToolsPtr visual_tools_;

        std::string name_;

    public:
        /**
         * \brief Constructor
         */
        CylindricalGraspGenerator()
                : name_("rviz_demo")
        {
          visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("base_link", "/grasp"));

          ROS_INFO("Sleeping 5 seconds before running demo");
          ros::Duration(2.0).sleep();

          // Clear messages
          visual_tools_->deleteAllMarkers();
          visual_tools_->enableBatchPublishing();
        }

        void generateGrasp(const Eigen::Affine3d& object_grasp_point, EigenSTL::vector_Affine3d& grasps)
        {
          Eigen::Affine3d grasp_pose;

          // Polar params
          const std::size_t polar_angle_count = 30;
          // Azimuthal params
          const std::size_t azimuthal_angle_count = 5;
          const double azimuthal_angle_min = - M_PI/180.0 * 15.0;
          const double azimuthal_angle_max = M_PI/180.0 * 15.0;
          const double azimuthal_angle_res = (azimuthal_angle_max - azimuthal_angle_min)/azimuthal_angle_count;

          // --------------------------------------------------------------------
          ROS_INFO_STREAM_NAMED(name_, "Object center");

          for (std::size_t polar_angle_idx = 0; polar_angle_idx < polar_angle_count; ++polar_angle_idx)
          {
            // Polar angle generation (rotation on Z axis)
            Eigen::Affine3d polar_rotation = Eigen::Affine3d::Identity() * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX())
                                             * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY())
                                             * Eigen::AngleAxisd(2*M_PI*polar_angle_idx/polar_angle_count, Eigen::Vector3d::UnitZ());

            for (std::size_t azimuthal_angle_idx = 0; azimuthal_angle_idx < azimuthal_angle_count; ++azimuthal_angle_idx)
            {
              // Azimuthal angle generation
              double azimuthal_angle = azimuthal_angle_min + azimuthal_angle_idx*azimuthal_angle_res;
              ROS_DEBUG_STREAM_NAMED(name_, "Azimuthal angle: " << azimuthal_angle);
              Eigen::Affine3d azimuthal_rotation = Eigen::Affine3d::Identity() * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX())
                                                   * Eigen::AngleAxisd(azimuthal_angle, Eigen::Vector3d::UnitY())
                                                   * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ());

              // Apply grasp displacement on X axis
              Eigen::Affine3d x_translation = Eigen::Affine3d::Identity();
              x_translation.translation().x() = 0.0;
              grasp_pose = object_grasp_point * polar_rotation * azimuthal_rotation * x_translation;
              // Add grasp point
              grasps.push_back(grasp_pose);
              visual_tools_->publishAxis(grasp_pose);
            }
          }
          visual_tools_->triggerBatchPublish();
        }

        /**
         * \brief Destructor
         */
        ~CylindricalGraspGenerator() {}
    };  // end class

}  // namespace rviz_visual_tools

int main(int argc, char** argv)
{
  ros::init(argc, argv, "visual_tools_demo");
  ROS_INFO_STREAM("Visual Tools Demo");

  // Allow the action server to recieve and send ros messages
  ros::AsyncSpinner spinner(1);
  spinner.start();

  hb_grasp_generator::CylindricalGraspGenerator demo;

  // Create pose
  Eigen::Affine3d object_center = Eigen::Affine3d::Identity() * Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitX())
                                  * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY())
                                  * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ());
  object_center.translation().x() = 1.0;
  EigenSTL::vector_Affine3d grasp;
  demo.generateGrasp(object_center, grasp);

  ROS_INFO_STREAM("Generated grasps: " << grasp.size());

  ROS_INFO_STREAM("Shutting down.");
  return 0;
}