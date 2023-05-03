#include <Eigen/Core>
#include <eigen_conversions/eigen_kdl.h>

#include <dh_kdl/util.hpp>
#include <dh_kdl/conversion/kdl_eigen.hpp>

#include "../../include/tobas_multirotor_controller/dynamics.hpp"

using namespace std;
using namespace KDL;

namespace tobas_multirotor_controller
{
MultiRotorDynamics::MultiRotorDynamics(const Tree& tree, const RotorConfigs& rotor_configs)
  : fk_solver_(tree), inertia_solver_(tree), ez_(0., 0., 1.), rotor_configs_(rotor_configs)
{
  assert(tree.getNrOfJoints() > 0);

  resize(STATE_SIZE, rotor_configs.size());
}

void MultiRotorDynamics::update(const double& roll, const double& pitch, const JntArray& q)
{
  updateA(roll, pitch);
  updateB(q);
}

void MultiRotorDynamics::updateA(const double& roll, const double& pitch)
{
  eulerrateFromAngvelLocal(roll, pitch, rpyvel_angvel_kdl_);
  tf::rotationKDLToEigen(rpyvel_angvel_kdl_, rpyvel_angvel_eigen_);
  A.block(0, 3, 3, 3) = rpyvel_angvel_eigen_;
}

void MultiRotorDynamics::updateB(const JntArray& q)
{
  inertia_solver_.JntToCart(q, P_base_cog_, I_cog_kdl_);
  tf::rotInertiaKDLToEigen(I_cog_kdl_, I_cog_eigen_);
  I_cog_eigen_.computeInverseWithCheck(I_cog_inv_, invertible_);
  assert(invertible_);

  for (int i = 0; i < rotor_configs_.size(); ++i)
  {
    fk_solver_.JntToCart(q, rotor_configs_[i].link_name, T_base_rotor_);
    P_cog_rotor_kdl_ = T_base_rotor_.p - P_base_cog_;
    tf::vectorKDLToEigen(P_cog_rotor_kdl_, P_cog_rotor_eigen_);
    const auto& d = rotor_configs_[i].direction;
    const auto& c = rotor_configs_[i].moment_constant;
    B.block(3, i, 3, 1) = I_cog_inv_ * (P_cog_rotor_eigen_.cross(ez_) - (d * c) * ez_);
  }
}
}  // namespace tobas_multirotor_controller
