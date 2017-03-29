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
    class GraspFilter
    {
    public:
        GraspFilter(robot_state::RobotState robot_state, moveit_visual_tools::MoveItVisualToolsPtr& visual_tools);
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
        CylindricalGraspGenerator();
        bool generateGrasp(const geometry_msgs::Pose& object_pose, std::vector<moveit_msgs::Grasp>& possible_grasps);

        /**
         * \brief Destructor
         */
        ~CylindricalGraspGenerator() {};
    };  // end class

}  // namespace hb_grasp_generator

#endif //HB_GRASP_GRASP_GENERATOR_H
