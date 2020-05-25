#ifndef UCHILE_GRASP_GRAVITATIONAL_TORQUE_ESTIMATION_H_
#define UCHILE_GRASP_GRAVITATIONAL_TORQUE_ESTIMATION_H_

#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>

#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>

#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jntarrayvel.hpp>
#include <kdl/chaindynparam.hpp>

namespace uchile_grasp_generator
{

class GravitationalTorqueEstimation
{
public:
  
  GravitationalTorqueEstimation(const std::shared_ptr<const urdf::ModelInterface>& robot_model, const std::string& root, const std::string& tip);
  bool estimate(const std::vector<double>& current_pos, std::vector<double>& torque_estimation);
  unsigned int getDOF();
  std::vector<std::string> getJointNames();
  double score(const std::vector<double>& position);

private:
  
  KDL::Chain kdl_chain_;
  KDL::JntArrayVel positions_;
  KDL::JntArray torques_;
  boost::shared_ptr<KDL::ChainDynParam> kdl_chain_dynamics_;
  unsigned int chain_dof_;
  std::vector<double> position_max_limits, position_min_limits;
};

typedef boost::shared_ptr<GravitationalTorqueEstimation> GravitationalTorqueEstimationPtr;

}  // namespace uchile_grasp_generator

#endif  // UCHILE_GRASP_GRAVITATIONAL_TORQUE_ESTIMATION_H_
