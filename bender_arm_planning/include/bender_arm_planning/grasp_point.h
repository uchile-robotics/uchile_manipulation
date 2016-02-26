/* 
* GraspPoint almacena un punto de grasp
*
* Autor: Rodrigo Munoz
* Junio 2015
*/

// Header guard
#ifndef BENDER_PLANNING_GRASP_POINT_
#define BENDER_PLANNING_GRASP_POINT_

// Libreria estandar
#include <vector>
#include <ostream>
// Serializacion
#include <boost/serialization/vector.hpp>
// Eigen para menejo de vectores
#include <Eigen/Core>
// Mensajes
#include <geometry_msgs/Point.h>
#include <bender_arm_planning/GraspPosition.h>

namespace capability_map{

  class GraspPoint
  {
    private:
      // Print
      friend std::ostream & operator<<(std::ostream &os, const GraspPoint &gp);
      // Permitir acceso a datos privados
      friend class boost::serialization::access;
      
      // Posicion
      float x_;
      float y_;
      float z_;

      // Soluciones IK
      std::vector< std::vector<float> > pregrasp_;
      std::vector< std::vector<float> > grasp_;

      // Serializacion
      template<class Archive>
      void serialize(Archive & ar, const unsigned int version){
        ar & x_ & y_ & z_ & pregrasp_ & grasp_;
      }

    public:
      // Contructores
      GraspPoint();

      GraspPoint(float x, float y, float z);

      GraspPoint(const Eigen::Vector3d& p);

      GraspPoint(const geometry_msgs::Point& p);

      bool addIkSolution(const std::vector<double>& pregrasp_sol, const std::vector<double>& grasp_sol);

      void setPosition(const Eigen::Vector3d& p);

      void getGraspPositionMsg(bender_arm_planning::GraspPosition& msg, const std::string ref_frame) const;

      void getPosition(Eigen::Vector3d& p) const;

      void getPosition(geometry_msgs::Point& p) const;

      std::size_t size() const;
  };

std::ostream & operator<<(std::ostream &os, const GraspPoint &gp);

} // capability_map namespace


// BENDER_PLANNING_GRASP_POINT_
#endif 
