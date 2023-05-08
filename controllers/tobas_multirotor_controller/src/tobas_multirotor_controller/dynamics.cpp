#include <Eigen/Core>
#include <eigen_conversions/eigen_kdl.h>

#include <dh_kdl/util.hpp>
#include <dh_kdl/conversion/kdl_eigen.hpp>

#include "../../include/tobas_multirotor_controller/dynamics.hpp"

#define Z_AXIS Vector3d(0., 0., 1.)

using namespace std;
using namespace KDL;
using namespace Eigen;

namespace tobas_multirotor_controller
{
MultiRotorDynamics::MultiRotorDynamics(const Drone& drone)
  : drone_(drone),
    ver_prop_idxes_(drone.rotorConfigIdxInAxis(Axis::Z_POSITIVE)),
    u_dim_(ver_prop_idxes_.size()),
    fk_solver_(drone.tree()),
    inertia_solver_(drone.tree())
{
  resize(STATE_SIZE, u_dim_);
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

  for (int i = 0; i < u_dim_; ++i)
  {
    const auto& rotor_idx = ver_prop_idxes_[i];
    const auto& rotor_config = drone_.rotorConfig(rotor_idx);

    fk_solver_.JntToCart(q, rotor_config.link_name, T_base_rotor_);
    P_cog_rotor_kdl_ = T_base_rotor_.p - P_base_cog_;
    tf::vectorKDLToEigen(P_cog_rotor_kdl_, P_cog_rotor_eigen_);
    const auto& d = rotor_config.direction;
    const auto& c = rotor_config.moment_constant;
    B.block(3, i, 3, 1) = I_cog_inv_ * (P_cog_rotor_eigen_.cross(Z_AXIS) - (d * c) * Z_AXIS);
  }
}
}  // namespace tobas_multirotor_controller
