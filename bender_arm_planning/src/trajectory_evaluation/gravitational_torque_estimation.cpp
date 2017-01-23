#include "gravitational_torque_estimation.h"

namespace trajectory_evaluation
{

GravitationalTorqueEstimation::GravitationalTorqueEstimation(
  const urdf::Model& robot_model,
  const std::string& root,
  const std::string& tip)
{
  // Load the tree
  KDL::Tree kdl_tree;
  if (!kdl_parser::treeFromUrdfModel(robot_model, kdl_tree))
  {
    ROS_ERROR("Could not construct KDL tree from URDF ");
    throw std::runtime_error("Could not construct KDL tree from URDF");
  }

  // Populate the Chain
  if(!kdl_tree.getChain(root, tip, kdl_chain_))
  {
    ROS_ERROR("Could not construct KDL chain from URDF");
    throw std::runtime_error("Could not construct KDL chain from URDF");
  }

  // Create chain using gravity vector
  kdl_chain_dynamics_.reset(new KDL::ChainDynParam(kdl_chain_, KDL::Vector(0,0,-9.81)));
  chain_dof_ = kdl_chain_.getNrOfJoints();
  // Set zero init positions and torques
  positions_ = KDL::JntArrayVel(chain_dof_);
  torques_ = KDL::JntArray(chain_dof_);
  KDL::SetToZero(positions_.q);
  KDL::SetToZero(positions_.qdot);
  KDL::SetToZero(torques_);
}


bool GravitationalTorqueEstimation::estimate(const std::vector<double>& current_pos, std::vector<double>& torque_estimation)
{
  // Check position vector size
  if (current_pos.size() != chain_dof_)
  {
    return false;
  }
  // Check output vector
  if (torque_estimation.size() != chain_dof_)
  {
    // Reset size and values
    torque_estimation.resize(chain_dof_, 0.0);
  }
  // Update current positions
  for (std::size_t i = 0; i < chain_dof_; ++i)
    positions_.q.data[i] = current_pos[i];

  // Gravity compensation
  kdl_chain_dynamics_->JntToGravity(positions_.q, torques_);

  // Update torque estimation
  for (std::size_t i = 0; i < chain_dof_; ++i)
    torque_estimation[i] = torques_.data[i];

  return true;
}

unsigned int GravitationalTorqueEstimation::getDOF()
{
  return chain_dof_;
}


std::vector<std::string> GravitationalTorqueEstimation::getJointNames()
{
  std::vector<std::string> names;
  for (std::size_t i = 0; i < chain_dof_; ++i)
    if (kdl_chain_.getSegment(i).getJoint().getType() != KDL::Joint::None)
      names.push_back(std::string(kdl_chain_.getSegment(i).getJoint().getName()));
  return names;
}

}  // namespace trajectory_evaluation


