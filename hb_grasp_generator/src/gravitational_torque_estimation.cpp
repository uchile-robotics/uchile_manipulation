#include "hb_grasp_generator/gravitational_torque_estimation.h"

namespace hb_grasp_generator
{

GravitationalTorqueEstimation::GravitationalTorqueEstimation(
  const boost::shared_ptr<const urdf::ModelInterface>& robot_model,
  const std::string& root,
  const std::string& tip)
{
  // Load the tree
  KDL::Tree kdl_tree;
  if (!kdl_parser::treeFromUrdfModel(*robot_model, kdl_tree))
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
  // Get joint limits
  std::vector<std::string> joint_names = getJointNames();
  position_min_limits.resize(chain_dof_, 0.0);
  position_max_limits.resize(chain_dof_, 0.0);
  std::size_t j = 0;
  for (std::vector<std::string>::iterator i = joint_names.begin(); i != joint_names.end(); ++i)
  {
    position_max_limits[j] = (robot_model->getJoint(*i)->limits->upper);
    position_min_limits[j++] = (robot_model->getJoint(*i)->limits->lower);
  }
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

double GravitationalTorqueEstimation::score(const std::vector<double>& position)
{
  if (position.size() != chain_dof_)
  {
    ROS_ERROR_STREAM("Joint position must have " << chain_dof_ << " elements (DOF).");
    return 0.0;
  }
  // Torque estimation & penalty
  double penalty_multiplier_ = 1.0;
  std::vector<double> penalty_ponderation(chain_dof_, 0.0);
  penalty_ponderation[0]=1;   // shoulder_pitch_joint
  penalty_ponderation[1]=1;   // shoulder_roll_joint
  penalty_ponderation[2]=1;   // shoulder_yaw_joint
  penalty_ponderation[3]=10;  // elbow_pitch_joint
  penalty_ponderation[4]=10;  // elbow_yaw_joint
  penalty_ponderation[5]=100; // wrist_pitch_joint

  double joint_limits_multiplier =1.0;
  double lower_bound_distance;
  double upper_bound_distance;
  std::vector<double> range(chain_dof_, 0.0);

  for (std::size_t i = 0; i < chain_dof_; ++i)
  {
    // Joint saturation
    double joint_sat = std::min(position_max_limits[i], std::max(position[i], position_min_limits[i]));
    upper_bound_distance = position_max_limits[i] - joint_sat;
    lower_bound_distance = joint_sat - position_min_limits[i];

    range[i] = position_max_limits[i] - position_min_limits[i];
    double joint_multiplier = (lower_bound_distance * upper_bound_distance / (range[i] * range[i]))*penalty_ponderation[i];

    // Saturation
    double joint_mult_sat = 0.25; // Demostrado al maximizar la formula de joint_multiplier
    if (joint_multiplier>joint_mult_sat)
    {
      joint_multiplier=joint_mult_sat;
    }

    joint_limits_multiplier *= joint_multiplier;

  }
  double position_penalty_index =  (1.0 - exp(-penalty_multiplier_ * joint_limits_multiplier))*1000.0/25.0;
  return position_penalty_index;
}

}  // namespace hb_grasp_generator


