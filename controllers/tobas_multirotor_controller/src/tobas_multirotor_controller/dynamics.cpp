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
MultiRotorDynamics::MultiRotorDynamics(const tobas::Drone& drone)
  : drone_(drone),
    fk_solver_(drone.tree()),
    inertia_solver_(drone.tree()),
    z_rotors_(drone, tobas::Axis::Z_POSITIVE)
{
  resize(STATE_SIZE, z_rotors_.count());
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

  for (uint32_t i = 0; i < z_rotors_.count(); ++i)
  {
    fk_solver_.JntToCart(q, z_rotors_.linkName(i), T_base_rotor_);
    P_cog_rotor_kdl_ = T_base_rotor_.p - P_base_cog_;
    tf::vectorKDLToEigen(P_cog_rotor_kdl_, P_cog_rotor_eigen_);
    const auto& d = z_rotors_.direction(i);
    const auto& c = z_rotors_.momentConstant(i);
    B.block(3, i, 3, 1) = I_cog_inv_ * (P_cog_rotor_eigen_.cross(Z_AXIS) - (d * c) * Z_AXIS);
  }
}
}  // namespace tobas_multirotor_controller
