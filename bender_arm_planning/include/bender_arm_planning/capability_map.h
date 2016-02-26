/* 
* Look at table para manipulacion
* Almacena GraspPoints en un grid rectangular, provee
* funciones de serializacion de datos y busqueda.
*
* Autor: Rodrigo Munoz
* Junio 2015
*/

// Header guard
#ifndef BENDER_PLANNING_CAPABILITY_MAP_
#define BENDER_PLANNING_CAPABILITY_MAP_

// Libreria estandar
#include <ostream>
// Serializacion
#include <boost/serialization/vector.hpp>
// Eigen para menejo de vectores
#include <Eigen/Core>
// GraspPoints
#include <bender_arm_planning/grasp_point.h>


namespace capability_map{

  class CapabilityMap
  {
  private:
      // Print
      friend std::ostream & operator<<(std::ostream &os, const CapabilityMap &gp);
      // Permitir acceso a datos privados
      friend class boost::serialization::access;
   
      // Serializacion
      template<class Archive>
      void serialize(Archive & ar, const unsigned int version)
      {
        ar & x_min_& x_max_;
        ar & y_min_& y_max_;
        ar & z_min_& z_max_;
        ar & Nx_ & Ny_ & Nz_;
        ar & x_cell_ & y_cell_ & z_cell_;
        ar & ref_frame_;
        ar & grasp_;
      }
      // Posiciones minimas y maximas
      float x_min_, x_max_;
      float y_min_, y_max_;
      float z_min_, z_max_;

      // Numero de celdas N^3
      unsigned int Nx_, Ny_, Nz_;

      // Tamano celdas
      float x_cell_, y_cell_, z_cell_;

      // Frame de referencia
      std::string ref_frame_;

  public:
      // Almacenar grasp
      std::vector<GraspPoint> grasp_;
   
  public:
      // Contructores
      CapabilityMap(const Eigen::Vector3d& x_i, const Eigen::Vector3d& x_f, 
        unsigned int Nx, unsigned int Ny, unsigned int Nz, const std::string& ref_frame);

      CapabilityMap();

      std::string getReferenceFrame() const;

      std::size_t getGraspSize(const Eigen::Vector3d& p) const;

      bool getIndex(const Eigen::Vector3d& p, std::size_t& index) const;

      bool getIndex(const geometry_msgs::Point& p, std::size_t& index) const;

      inline bool getGridPosition(Eigen::Vector3d& p, const unsigned int& i) const
      {
        p[0] = x_min_ + x_cell_*(i/(Ny_*Nz_));
        
        unsigned int j = i%(Ny_*Nz_);
        p[1] = y_min_ + y_cell_*(j/Nz_);

        unsigned int k = j%Nz_;
        p[2] = z_min_ + z_cell_*k;
        // Check intervalo
        if (p[0]<x_min_ || p[0]>(x_max_+x_cell_))
          return false;
        if (p[1]<y_min_ || p[1]>(y_max_+y_cell_))
          return false;
        if (p[2]<z_min_ || p[2]>(z_max_+z_cell_))
          return false;
        return true;
      }

      void addGrasp(GraspPoint& grasp);

      bool getGrasp(const geometry_msgs::Point& p, GraspPoint& gp) const;

      bool getGrasp(const Eigen::Vector3d& p, GraspPoint& gp) const;

      bool getGrasp(const Eigen::Vector3d& p, std::vector<GraspPoint>& gp) const;

      double getResolution(char axis = 'z') const;
  };

  typedef boost::shared_ptr<CapabilityMap> CapabilityMapPtr;

  std::ostream & operator<<(std::ostream &os, const CapabilityMap &cm);

  CapabilityMapPtr loadCapMap(const std::string &filename);

  bool saveCapMap(CapabilityMapPtr cm, const std::string &filename);
  

} // capability_map namespace

// BENDER_PLANNING_CAPABILITY_MAP_
#endif 
