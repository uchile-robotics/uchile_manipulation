/* 
* Permite generar un mapa con las capacidades de grasp
* para objetos cilindricos en el espacio.
*
* Autor: Rodrigo Munoz
*/

// ROS
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/common_planning_interface_objects/common_objects.h>
// Visual
#include <moveit_visual_tools/moveit_visual_tools.h>
// Grasp
#include <moveit_simple_grasps/simple_grasps.h>
#include <moveit_simple_grasps/grasp_data.h>
#include <moveit_simple_grasps/grasp_filter.h>
// Capability map
#include <bender_arm_planning/capability_map.h>
#include <bender_arm_planning/grasp_point.h>

namespace bender_arm_planning
{

using namespace moveit::planning_interface;

class CapabilityMapGenerator {
  private:
    // ROS Nodehandle
    ros::NodeHandle nh_;

    // TF Listener
    boost::shared_ptr<tf::Transformer> tf_;

    // Planning scene
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

    // Herramienta para generar y visualizar
    moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;
    // Grasp filter
    moveit_simple_grasps::GraspFilterPtr grasp_filter_;
    // Grasp generator
    moveit_simple_grasps::SimpleGraspsPtr simple_grasps_;

    // Capability Map
    capability_map::CapabilityMapPtr capability_map_;
    
    // Datos del gripper
    moveit_simple_grasps::GraspData grasp_data_;
    // Robot model
    robot_model::RobotModelPtr robot_model_;
    // Robot state, estado cinematico del robot
    robot_state::RobotStatePtr robot_state_;

    // Logger
    const std::string name_;
    
  public:
    // Nombre brazo y gripper
    const std::string arm_name_;
    const std::string gripper_name_;
    // Frames de referencia
    std::string ref_frame_;
    // Frames efector
    std::string ee_frame_;
    // Nombre archivo
    std::string file_name_;
    // Nombres de joints
    std::vector<std::string> joint_names_;
  
  public:
    CapabilityMapGenerator(const std::string& arm_name, const std::string& gripper_name):
      nh_("~"),
      name_("capability_map"),
      arm_name_(arm_name),
      gripper_name_(gripper_name)
    {
      // TF
      tf_ = getSharedTF();

      // Cargar planning scene monitor
      ROS_INFO("Loading Planning Scene Monitor...");
      planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description", tf_));
      ros::spinOnce();
      ros::Duration(0.5).sleep();
      ros::spinOnce();


      // Herramientas para generar y visualizar
      ROS_INFO("Visual tools on topic /markers");
      visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools("bender/base_link","/markers", planning_scene_monitor_));
      
      // ERROR EN LA API!, se modifica mientras, pero debe ser revisado
      visual_tools_->loadEEMarker(grasp_data_.ee_group_);
      //visual_tools_->loadEEMarker(grasp_data_.ee_group_, arm_name_);
    
      // Generador de grasp
      simple_grasps_.reset( new moveit_simple_grasps::SimpleGrasps(visual_tools_) );
      robot_state::RobotState robot_state = planning_scene_monitor_->getPlanningScene()->getCurrentState();
      grasp_filter_.reset(new moveit_simple_grasps::GraspFilter(robot_state, visual_tools_) );

      // Capability Map

      // Parametros de dimensiones del cap map
      std::vector<double> init_pos, final_pos;
      if (nh_.hasParam("init_pos"))
      {
        nh_.getParam("init_pos", init_pos);
      }
      else
      {
        ROS_WARN("Parameter 'init_pos' missing from parameter server. Using default.");
        init_pos.push_back(0.3);
        init_pos.push_back(0.2);
        init_pos.push_back(0.7);
      }
      if (nh_.hasParam("final_pos"))
      {
        nh_.getParam("final_pos", final_pos);
      }
      else
      {
        ROS_WARN("Parameter 'final_pos' missing from parameter server. Using default.");
        final_pos.push_back(0.5);
        final_pos.push_back(0.3);
        final_pos.push_back(1.0);
      }

      // Resolucion
      std::vector<int> res;
      if (nh_.hasParam("resolution"))
      {
        nh_.getParam("resolution", res);
      }
      else
      {
        ROS_WARN("Parameter 'resolution' missing from parameter server. Using default.");
        res.push_back(1);
        res.push_back(3);
        res.push_back(3);
      }

      // Frame de referencia
      if (nh_.hasParam("ref_frame"))
      {
        nh_.getParam("ref_frame", ref_frame_);
        ROS_INFO("Using ref frame: %s", ref_frame_.c_str());
      }
      else
      {
        ROS_WARN("Parameter 'ref_frame' missing from parameter server. Using default.");
        ref_frame_ = "bender/base_link";
      }

      // Nombre de archivo
      if (nh_.hasParam("file_name"))
      {
        nh_.getParam("file_name", file_name_);
        ROS_INFO("Using file name: %s", file_name_.c_str());
      }
      else
      {
        ROS_WARN("Parameter 'file_name' missing from parameter server. Using default.");
        file_name_ = "store.dat";
      }

      capability_map_.reset(new capability_map::CapabilityMap(
        Eigen::Vector3d(init_pos[0], init_pos[1], init_pos[2]),
        Eigen::Vector3d(final_pos[0], final_pos[1], final_pos[2]),
        res[0], res[1], res[2], ref_frame_));
      
    }

    capability_map::CapabilityMapPtr getCapabilityMap()
    {
      return capability_map_;
    }

    bool loadData()
    {
      // Cargar grasp data
      ros::NodeHandle nh_p("~");
      ROS_INFO_NAMED(name_, "Loading grasp data for %s in namespace %s", gripper_name_.c_str(), nh_p.getNamespace().c_str());
      
      return grasp_data_.loadRobotGraspData(nh_p, gripper_name_);
    }

    bool generateMap()
    {
      unsigned int N = capability_map_->grasp_.size();
      for (unsigned int i = 0; i < N; ++i)
      {
        ROS_WARN("Generando Grasp [%i/%i] p%.2f", i, N-1, 100.0*i/(N-1));
        generateGrasp(capability_map_->grasp_[i]);
        if (!ros::ok())
        {
          return false;
        }
      }
      return true;
    }

    bool generateGrasp(capability_map::GraspPoint& grasp)
    {
      // Obtener posicion de grasp
      geometry_msgs::Pose target_pose;
      target_pose.orientation.x = 0.0;
      target_pose.orientation.y = 0.0;
      target_pose.orientation.z = 0.0;
      target_pose.orientation.w = 1.0;
      grasp.getPosition(target_pose.position);

      std::vector<trajectory_msgs::JointTrajectoryPoint> ik_solutions;
      std::vector<moveit_msgs::Grasp> possible_grasps;

      bool verbose = false;

      double target_height = 0.1;
      double target_radius = 0.05;
      double angle_resolution = 30;
      // Publicar markers de objeto
      visual_tools_->setLifetime(5.0);
      visual_tools_->publishCylinder(target_pose, rviz_visual_tools::RAND);
      /* --------------------------------------------------------------------------------
      * Parametros de grasping
      * 
      */
      grasp_data_.angle_resolution_ = angle_resolution; // Particiones de eje axial
      grasp_data_.approach_retreat_desired_dist_ = std::min(target_radius + 0.06, 0.1); // Distancia de approach y retreat
      //ROS_INFO_STREAM("Distancia approach " << grasp_data_.approach_retreat_desired_dist_);
      grasp_data_.approach_retreat_min_dist_ = 0.1;

      /* --------------------------------------------------------------------------------
      * Generar grasps
      * Genera distintas posiciones de grasp y pregrasp para objetos cilindricos
      * @TODO: Manejo de distintas orientaciones
      *
      */
      // Generar grasps en eje z desde -M_PI/2 a M_PI/2
      simple_grasps_->generateAxisGrasps(target_pose, moveit_simple_grasps::Z_AXIS, moveit_simple_grasps::UP,
          moveit_simple_grasps::HALF, 0, grasp_data_, possible_grasps);
      if (possible_grasps.empty())
      {
        ROS_ERROR_NAMED(name_, "No grasp generated!");
        return false;
      }
      //ROS_INFO_STREAM_NAMED(name_, "Generated " << possible_grasps.size() << " grasps." );
      
      /* --------------------------------------------------------------------------------
      * Filtrar usando IK
      * Cada posicion de grasp y pregrasp es filtrada comprobando su factibilidad usando 
      * cinematica inversa (IK). 
      * @TODO:
      * - Realizar chequeo de colisiones de estados de grasp y pregrasp
      * - Anadir bitset para saber cuales cumplen con IK en pregrasp
      *
      */
      bool filter_pregrasps = true;
      //ROS_INFO_STREAM_NAMED(name_, "Filtering using IK ");
      grasp_filter_->filterGrasps(possible_grasps, ik_solutions, filter_pregrasps, grasp_data_.ee_parent_link_, arm_name_);
      if (ik_solutions.empty())
      {
        ROS_WARN_NAMED(name_, "No grasp pass the IK filter, grasp not reachable.");
        return false;
      }
      ROS_INFO_STREAM_NAMED(name_, "Found " << possible_grasps.size() << " possible grasp." );
      // Mostrar soluciones factibles
      if (verbose)
      {
        // ERROR API: comantado para poder compilar. REVISAR!
        //visual_tools_->publishIKSolutions(ik_solutions, arm_name_, 0.05);
      }

      for (unsigned int i = 0; i < ik_solutions.size()-1; ++i)
      {
        std::vector<double>& grasp_ref = ik_solutions[i].positions;
        std::vector<double>& pregrasp_ref = ik_solutions[++i].positions;
        grasp.addIkSolution(pregrasp_ref, grasp_ref);
      }
      return true;
    }

    void save()
    {
      if (!capability_map::saveCapMap(capability_map_,file_name_))
      {
        ROS_ERROR("Error saving file!");
      }
    }



  }; // Clase CapabilityMapGenerator



  typedef boost::shared_ptr<CapabilityMapGenerator> CapabilityMapGeneratorPtr;

} // bender_arm_planning namespace

int main(int argc, char **argv)
{
  /*
  using namespace capability_map;

  GraspPoint grasp(0.09,0.09,0.09);
  std::vector<double> test_ik(6, 0.0);
  grasp.addIkSolution(test_ik, test_ik);

  GraspPoint grasp2(0.91,0.91,0.91);
  std::vector<double> test_ik2(6, 0.3);
  grasp2.addIkSolution(test_ik, test_ik);
  grasp2.addIkSolution(test_ik2, test_ik2);

  CapabilityMapPtr cm(new CapabilityMap(Eigen::Vector3d(0.0,0.0,0.0), Eigen::Vector3d(1.0,1.0,1.0), 100, 100, 100, "base_link"));
  cm->addGrasp(grasp);
  cm->addGrasp(grasp2);
  // Guardar
  saveCapMap(cm,"store.dat");


  CapabilityMapPtr restored_cm = loadCapMap("store.dat");
  if(!restored_cm)
  {
    ROS_ERROR("Error loading file");
  }
  std::cout << *cm << std::endl;
  std::cout << "\033[92m" << "Restored" << "\033[0m" << std::endl;
  std::cout << *restored_cm << std::endl;

  
  return 0;
  */

  ros::init(argc, argv, "capability_map_gen");
  ros::AsyncSpinner spinner(2);
  spinner.start();

  ros::Duration(5.0).sleep();
  using namespace bender_arm_planning;
  CapabilityMapGeneratorPtr cmg(new CapabilityMapGenerator("l_arm", "l_gripper"));
  if(!cmg->loadData())
  {
    ROS_ERROR("Error loading grasp data");
    ros::shutdown();
    exit(EXIT_FAILURE);
  }
  cmg->generateMap();
  ROS_WARN("Saving File!");
  cmg->save();
  
  return 0;
}