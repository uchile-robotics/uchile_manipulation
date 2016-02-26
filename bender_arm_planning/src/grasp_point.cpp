#include <bender_arm_planning/grasp_point.h>


namespace capability_map{

  GraspPoint::GraspPoint():
  x_(0.0), y_(0.0), z_(0.0)
  {};

  GraspPoint::GraspPoint(float x, float y, float z): 
  x_(x), y_(y), z_(z)
  {};

  GraspPoint::GraspPoint(const Eigen::Vector3d& p): 
  x_(p[0]), y_(p[1]), z_(p[2])
  {};

  GraspPoint::GraspPoint(const geometry_msgs::Point& p):
  x_(p.x), y_(p.y), z_(p.z)
  {};

  bool GraspPoint::addIkSolution(const std::vector<double>& pregrasp_sol, const std::vector<double>& grasp_sol)
  {
    // Check largo
    if (pregrasp_sol.size() != grasp_sol.size())
    {
      return false;
    }
    // Anadir

    std::vector<float> pregrasp_sol_f(pregrasp_sol.begin(), pregrasp_sol.end());
    std::vector<float> grasp_sol_f(grasp_sol.begin(), grasp_sol.end());

    pregrasp_.push_back(pregrasp_sol_f);
    grasp_.push_back(grasp_sol_f);
    return true;
  }

  void GraspPoint::setPosition(const Eigen::Vector3d& p)
  {
    // Posicion
    x_ = p(0);
    y_ = p(1);
    z_ = p(2);
  }

  void GraspPoint::getPosition(Eigen::Vector3d& p) const
  {
    // Posicion
    p(0) = x_;
    p(1) = y_;
    p(2) = z_;
  }

  
  void GraspPoint::getPosition(geometry_msgs::Point& p) const
  {
    // Posicion
    p.x = x_;
    p.y = y_;
    p.z = z_;
  }

  std::size_t GraspPoint::size() const
  {
    return grasp_.size();
  }

  void GraspPoint::getGraspPositionMsg(bender_arm_planning::GraspPosition& msg, const std::string ref_frame) const
  {
    // Pose
    msg.grasp_pose.pose.position.x = x_;
    msg.grasp_pose.pose.position.y = y_;
    msg.grasp_pose.pose.position.z = z_;
    msg.grasp_pose.pose.orientation.w = 1.0;
    msg.grasp_pose.header.frame_id = ref_frame;
    // Posiciones
    msg.grasp.resize(grasp_.size()); msg.pregrasp.resize(pregrasp_.size());
    for (int i = 0; i < grasp_.size(); ++i)
    {
      /*
      std::cout << "Grasp ";
      for (std::vector<float>::const_iterator j = grasp_[i].begin(); j != grasp_[i].end(); ++j)
        std::cout << *j << ' ';
      std::cout << std::endl << "Pregrasp ";
      for (std::vector<float>::const_iterator j = pregrasp_[i].begin(); j != pregrasp_[i].end(); ++j)
        std::cout << *j << ' ';
      std::cout << std::endl << "---" << std::endl;
      */
      
      msg.grasp[i].positions.assign(grasp_[i].begin(), grasp_[i].end());
      msg.pregrasp[i].positions.assign(pregrasp_[i].begin(), pregrasp_[i].end());
    }
  }

  std::ostream & operator<<(std::ostream &os, const GraspPoint &gp)
  {
    return os << "[" << gp.x_ << "," << gp.y_ << "," << gp.z_ << "] N " << gp.pregrasp_.size();
  }

} // capability_map namespace