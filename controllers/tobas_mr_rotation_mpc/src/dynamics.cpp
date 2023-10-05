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
  // 慣性テンソルと重心を計算
  auto I_base = inertia_solver_.JntToCart(q);
  const auto P_base_cog = I_base.getCOG();
  const auto I_cog = I_base.RefPoint(P_base_cog).getRotationalInertia();
  tf::rotInertiaKDLToEigen(I_cog, I_cog_);
  const Matrix3d I_cog_inv = I_cog_.inverse();

  // Update A
  eulerrateFromAngvelLocal(roll, pitch, rpyvel_angvel_kdl_);
  tf::rotationKDLToEigen(rpyvel_angvel_kdl_, rpyvel_angvel_eigen_);
  A.block<3, 3>(kRotIdx, kGyroIdx) = rpyvel_angvel_eigen_;
  A.block<3, 3>(kGyroIdx, kHForceIdx) = I_cog_inv;

  // Update B
  for (uint32_t i = 0; i < z_rotors_.count(); ++i)
  {
    const auto T_base_rotor = fk_solver_.JntToCart(q, z_rotors_.linkName(i));
    const auto P_cog_rotor = T_base_rotor.p - P_base_cog;
    tf::vectorKDLToEigen(P_cog_rotor, P_cog_rotor_);
    const auto& d = z_rotors_.direction(i);
    const auto& cm = z_rotors_.momentConstant(i);
    B.block<3, 1>(kGyroIdx, i) = I_cog_inv * (P_cog_rotor_.cross(UNIT_Z) - (d * cm) * UNIT_Z);
  }
}
}  // namespace tobas_mr_rotation_mpc
