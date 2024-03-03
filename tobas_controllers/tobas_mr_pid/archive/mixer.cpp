#include <Eigen/SVD>
#include <eigen_conversions/eigen_kdl.h>

#include <dh_kdl/util.hpp>
#include <dh_kdl/conversion/kdl_eigen.hpp>

#include "../include/tobas_mr_pid/mixer.hpp"

#define UNIT_Z Vector3d::UnitZ()

using namespace std;
using namespace Eigen;
using namespace KDL;

namespace tobas_mr_pid
{
Mixer::Mixer(const tobas::Drone& drone)
  : drone_(drone),
    fk_solver_(drone.tree()),
    inertia_solver_(drone.tree()),
    z_rotors_(drone, tobas::Axis::Z_POSITIVE)
{
  updateInternalDataStructures();
}

void Mixer::updateInternalDataStructures()
{
  fk_solver_.updateInternalDataStructures();
  inertia_solver_.updateInternalDataStructures();
  z_rotors_.updateInternalDataStructures();

  A_.resize(NoChange, z_rotors_.count());
  A_.row(0).setOnes();
}

VectorXd Mixer::solve(
  const Vector& cur_gyro_B,
  const Vector& tar_dgyro_B,
  const JntArray& q,
  const double& tar_thrust_sum)
{
  assert(tar_thrust_sum >= 0);

  // 重心と慣性テンソルを計算
  inertia_solver_.JntToCart(q, P_base_cog_, I_cog_kdl_);
  tf::rotInertiaKDLToEigen(I_cog_kdl_, I_cog_eigen_);

  // Aを更新
  for (uint32_t i = 0; i < z_rotors_.count(); ++i)
  {
    fk_solver_.JntToCart(q, z_rotors_.linkName(i), T_base_rotor_);
    const auto P_cog_rotor_kdl = T_base_rotor_.p - P_base_cog_;
    tf::vectorKDLToEigen(P_cog_rotor_kdl, P_cog_rotor_eigen_);
    const auto& d = z_rotors_.direction(i);
    const auto& cm = z_rotors_.momentConstant(i);
    A_.block<3, 1>(1, i) = P_cog_rotor_eigen_.cross(UNIT_Z) - (d * cm) * UNIT_Z;
  }

  // bを更新
  // TODO: H-forceを考慮
  tf::vectorKDLToEigen(cur_gyro_B, cur_gyro_eigen_);
  tf::vectorKDLToEigen(tar_dgyro_B, tar_dgyro_eigen_);
  const auto inertia_force = I_cog_eigen_ * tar_dgyro_eigen_;
  const auto coriolis_force = cur_gyro_eigen_.cross(I_cog_eigen_ * cur_gyro_eigen_);
  b_(0) = tar_thrust_sum;
  b_.block<3, 1>(1, 0) = inertia_force + coriolis_force;

  // 最小二乗解を計算
  return A_.jacobiSvd(ComputeThinU | ComputeThinV).solve(b_);
}
}  // namespace tobas_mr_pid
