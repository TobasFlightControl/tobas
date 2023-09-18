#include <Eigen/LU>
#include <eigen_conversions/eigen_kdl.h>

#include <dh_kdl/util.hpp>
#include <dh_kdl/conversion/kdl_eigen.hpp>

#include "../include/tobas_mr_rotation_mpc/dynamics.hpp"
#include "../include/tobas_mr_rotation_mpc/constants.hpp"

#define UNIT_Z Vector3d::UnitZ()

using namespace std;
using namespace Eigen;
using namespace KDL;

namespace tobas_mr_rotation_mpc
{
MultiRotorDynamics::MultiRotorDynamics(const tobas::Drone& drone)
  : drone_(drone),
    fk_solver_(drone.tree()),
    inertia_solver_(drone.tree()),
    z_rotors_(drone, tobas::Axis::Z_POSITIVE)
{
  resize(kStateSize, z_rotors_.count());
}

void MultiRotorDynamics::updateInternalDataStructures()
{
  fk_solver_.updateInternalDataStructures();
  inertia_solver_.updateInternalDataStructures();
  z_rotors_.updateInternalDataStructures();

  resize(kStateSize, z_rotors_.count());
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
  A.block<3, 3>(0, 3) = rpyvel_angvel_eigen_;
}

void MultiRotorDynamics::updateB(const JntArray& q)
{
  inertia_solver_.JntToCart(q, P_base_cog_, I_cog_kdl_);
  tf::rotInertiaKDLToEigen(I_cog_kdl_, I_cog_eigen_);
  const PartialPivLU<Matrix3d> I_cog_lu(I_cog_eigen_);

  for (uint32_t i = 0; i < z_rotors_.count(); ++i)
  {
    fk_solver_.JntToCart(q, z_rotors_.linkName(i), T_base_rotor_);
    const auto P_cog_rotor_kdl = T_base_rotor_.p - P_base_cog_;
    tf::vectorKDLToEigen(P_cog_rotor_kdl, P_cog_rotor_eigen_);
    const auto& d = z_rotors_.direction(i);
    const auto& c = z_rotors_.momentConstant(i);
    B.block<3, 1>(3, i) = I_cog_lu.solve(P_cog_rotor_eigen_.cross(UNIT_Z) - (d * c) * UNIT_Z);
  }
}
}  // namespace tobas_mr_rotation_mpc
