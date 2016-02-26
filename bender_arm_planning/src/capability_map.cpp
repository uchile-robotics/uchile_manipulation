// Libreria estandar
#include <fstream>
// Serializacion
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/filesystem.hpp>
// Herramientas para archivos
#include <boost/filesystem.hpp>

#include <bender_arm_planning/capability_map.h>

namespace capability_map{


  CapabilityMap::CapabilityMap(const Eigen::Vector3d& x_i, const Eigen::Vector3d& x_f, 
    unsigned int Nx, unsigned int Ny, unsigned int Nz, const std::string& ref_frame):
  x_min_(x_i[0]), x_max_(x_f[0]),
  y_min_(x_i[1]), y_max_(x_f[1]),
  z_min_(x_i[2]), z_max_(x_f[2]),
  Nx_(Nx), Ny_(Ny), Nz_(Nz),
  x_cell_((x_max_-x_min_)/Nx_),
  y_cell_((y_max_-y_min_)/Ny_),
  z_cell_((z_max_-z_min_)/Nz_),
  ref_frame_(ref_frame)
  {
    // Aumentar capacidad
    grasp_.reserve(Nx_*Ny_*Nz_);
    // Anadir elementos
    Eigen::Vector3d p;
    for (unsigned int i = 0; i < Nx_*Ny_*Nz_; ++i)
    {
      // Obtener posicion
      getGridPosition(p, i); // inline, implementada en header
      grasp_.push_back(GraspPoint(p));
    }
  }

  CapabilityMap::CapabilityMap() {}

  std::string CapabilityMap::getReferenceFrame() const
  {
    return ref_frame_;
  }

  std::size_t CapabilityMap::getGraspSize(const Eigen::Vector3d& p) const
  {
    std::size_t i;
    if (getIndex(p, i))
    {
      return grasp_[i].size();
    }
    else
    {
      return 0;
    }
  }

  // Funcion de busqueda
  bool CapabilityMap::getIndex(const Eigen::Vector3d& p, std::size_t& index) const
  {
    // Check intervalo
    if (p[0]<x_min_ || p[0]>(x_max_+x_cell_/2))
      return false;
    if (p[1]<y_min_ || p[1]>(y_max_+y_cell_/2))
      return false;
    if (p[2]<z_min_ || p[2]>(z_max_+z_cell_/2))
      return false;
    unsigned int x_ind = (p[0]-x_min_)/x_cell_;
    unsigned int y_ind = (p[1]-y_min_)/y_cell_;
    unsigned int z_ind = (p[2]-z_min_)/z_cell_;
    index = z_ind + y_ind*Nz_ + x_ind*Ny_*Nz_;
    if (index > grasp_.size())
    {
      //std::cerr << "\033[91m" << "Out of workspace" << "\033[0m" << std::endl;
      return false;
    }
    return true;
  }

  bool CapabilityMap::getIndex(const geometry_msgs::Point& p, std::size_t& index) const
  {
    // Check intervalo
    Eigen::Vector3d v(p.x, p.y, p.z);
    return getIndex(v, index);
  }

  void CapabilityMap::addGrasp(GraspPoint& grasp)
  {
    // Obtener posicion de grasp
    Eigen::Vector3d p;
    grasp.getPosition(p);

    // Obtener indice en la grilla
    std::size_t i(0);
    if ( getIndex(p, i))
    {
      // Obtener posicion en grilla
      getGridPosition(p, i); // @TODO implementar roundPosition
      grasp.setPosition(p);
      std::cout << "Grasp " << grasp << " index " << i << std::endl;
      // Anadir grasp en la posicion adecuada de la grilla
      grasp_[i]=grasp;
    }
    else
    {
      std::cerr << "\033[91m" << "Out of workspace" << "\033[0m" << std::endl;
    }
  }

  bool CapabilityMap::getGrasp(const geometry_msgs::Point& p, GraspPoint& gp) const
  {
    Eigen::Vector3d pos(p.x, p.y, p.z);
    return CapabilityMap::getGrasp(pos, gp);
  }

  bool CapabilityMap::getGrasp(const Eigen::Vector3d& p, GraspPoint& gp) const
  {
    std::size_t i;
    if (getIndex(p, i))
    {
      if (grasp_[i].size()>0){
        gp = grasp_[i];
        return true;
      }
    }
    return false;
  }

  bool CapabilityMap::getGrasp(const Eigen::Vector3d& p, std::vector<GraspPoint>& gp) const
  {
    std::size_t i;
    if (getIndex(p, i))
    {
      if (grasp_[i].size()>0){
         gp.push_back(grasp_[i]);
        return true;
      }
    }
    return false;
  }

  double CapabilityMap::getResolution(char axis) const
  {
    switch (axis)
    {
      case 'z':
        return z_cell_;
      case 'x':
        return x_cell_;
      case 'y':
        return y_cell_;
      default:
        return z_cell_;
    }
  }

  std::ostream & operator<<(std::ostream &os, const CapabilityMap &cm)
  {
    os << "Resolution: [" << cm.Nx_ << ", " << cm.Ny_ << ", " << cm.Nz_ << "]" << std::endl;
    os << "Ref frame: " << cm.ref_frame_ << std::endl;
    for (std::vector<GraspPoint>::const_iterator gp = cm.grasp_.begin(); gp != cm.grasp_.end(); ++gp)
    {
      os << *gp << std::endl;
    }
    return os;
  }

  CapabilityMapPtr loadCapMap(const std::string &filename)
  {
    // Check existencia
    CapabilityMapPtr cm;
    if ( !boost::filesystem::exists(filename.c_str()) )
    {
      std::cerr << "\033[91m" << "File " << filename << " dont exists." << "\033[0m" << std::endl;
      return cm; // Null
    }
    std::ifstream ifs(filename.c_str());
    if (ifs.is_open())
    {
      cm.reset(new CapabilityMap);
      boost::archive::text_iarchive ia(ifs);
      ia >> *cm;
      std::cout << "\033[92m" << "Capability Map file loaded. File: " << filename << "\033[0m" << std::endl;
    }
    else
    {
      std::cerr << "\033[91m" << "Error opening " << filename << "\033[0m" << std::endl;
    } 
    ifs.close();
    return cm;
  }

  bool saveCapMap(CapabilityMapPtr cm, const std::string &filename)
  {
    namespace bf = boost::filesystem;

    bf::path path(filename);
    bf::path full_path = bf::complete(path);

    std::cout << "Saving file in " << full_path << std::endl;
    std::ofstream ofs(filename.c_str());
    if (ofs.is_open())
    {
      boost::archive::text_oarchive oa(ofs);
      oa << *cm;
    }
    else
    {
      // Error al guardar archivo
      return false;
    }
    ofs.close();
    // Check archivo
    if( !bf::exists(full_path) ) {
      std::cerr << "\033[91m" << "Could not find: " << path.string() << "\033[0m" << std::endl;
      return false;
    }
    float size = 1.0*(static_cast<int>(bf::file_size(full_path)))/1000.0;
    std::cout << "CapabilityMap Size " << size << " kB"<< std::endl;
    return true;
  }

} // capability_map namespace

/*
int main(int argc, char** argv) 
{
  using namespace capability_map;

  GraspPoint grasp(0.09,0.09,0.09);
  std::vector<double> test_ik(6, 0.0);
  grasp.add_ik_solution(test_ik, test_ik);

  GraspPoint grasp2(0.91,0.91,0.91);
  std::vector<double> test_ik2(6, 0.3);
  grasp2.add_ik_solution(test_ik, test_ik);
  grasp2.add_ik_solution(test_ik2, test_ik2);

  CapabilityMap cm(Eigen::Vector3d(0.0,0.0,0.0), Eigen::Vector3d(1.1,1.1,1.1), 3, 3, 3, "base_link");
  cm.add_grasp(grasp);
  cm.add_grasp(grasp2);
  // Guardar
  cm.save("store.dat");

  // Restaurar
  CapabilityMapPtr restored_cm_ptr(new CapabilityMap);
  restored_cm_ptr->load("store.dat");

  std::cout << cm << std::endl;
  std::cout << "Restored" << std::endl;
  std::cout << *restored_cm_ptr << std::endl;
  
  return 0;
}
*/