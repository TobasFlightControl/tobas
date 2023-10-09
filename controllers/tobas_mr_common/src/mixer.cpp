#include <tobas_tools/constants.hpp>

#include "../include/tobas_mr_common/mixer.hpp"

#define UNIT_Z Vector3d::UnitZ()

using namespace std;
using namespace Eigen;
using namespace KDL;

namespace tobas_mr_common
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

  qp_solver_.resize(3 + z_rotors_.count(), kEqualityConstSize, z_rotors_.count() * 2);
  qp_solver_.setZero();

  // QPの決定変数のスケール
  constexpr double dgyro_scale = M_PI;
  const auto thrust_scale = inertia_solver_.JntToMass() * tobas::kGravity / z_rotors_.count();
  qp_solver_.x_scale.head(3).fill(dgyro_scale);
  qp_solver_.x_scale.tail(z_rotors_.count()).fill(thrust_scale);

  // QPPの定数部分
  qp_solver_.problem.G.bottomRightCorner(1, z_rotors_.count()).fill(1);
  qp_solver_.problem.A.topRightCorner(z_rotors_.count(), z_rotors_.count()).diagonal().fill(1);
  qp_solver_.problem.A.bottomRightCorner(z_rotors_.count(), z_rotors_.count()).diagonal().fill(-1);

  // QPの重み
  updateQpWeight();

  A_.resize(NoChange, z_rotors_.count());
  max_thrusts_.resize(z_rotors_.count());
  min_thrusts_.resize(z_rotors_.count());
}

VectorXd Mixer::solve(
  const double& cur_voltage,
  const JntArray& cur_q,
  const Vector3d& cur_gyro_B,
  const Vector3d& tar_dgyro_B,
  const VectorXd& tar_thrusts)
{
  // 慣性テンソルと重心を計算
  auto I_base = inertia_solver_.JntToCart(cur_q);
  const auto P_base_cog = I_base.getCOG();
  const auto I_cog = I_base.RefPoint(P_base_cog).getRotationalInertia();

  for (uint32_t i = 0; i < z_rotors_.count(); ++i)
  {
    const auto T_base_rotor = fk_solver_.JntToCart(cur_q, z_rotors_.linkName(i));
    const auto P_cog_rotor = T_base_rotor.p - P_base_cog;
    const auto& d = z_rotors_.direction(i);
    const auto& cm = z_rotors_.momentConstant(i);
    A_.col(i) = (d * cm) * UNIT_Z - P_cog_rotor.data.cross(UNIT_Z);
  }

  qp_solver_.problem.G.topLeftCorner(3, 3) = I_cog.data;
  qp_solver_.problem.G.topRightCorner(3, z_rotors_.count()) = A_;

  // TODO: H-forceを考慮
  const auto inertia_torque = I_cog.data * tar_dgyro_B;
  const auto coriolis_torque = cur_gyro_B.cross(I_cog.data * cur_gyro_B);
  qp_solver_.problem.h.head(3) = -inertia_torque - coriolis_torque - A_ * tar_thrusts;

  const auto min_voltage = cur_voltage * tobas::kMotorSpinArm;
  for (uint32_t i = 0; i < z_rotors_.count(); ++i)
  {
    max_thrusts_(i) = z_rotors_.thrustFromVoltage(i, cur_voltage);
    min_thrusts_(i) = z_rotors_.thrustFromVoltage(i, min_voltage);
  }

  const auto max_dthrusts = max_thrusts_ - tar_thrusts;
  const auto min_dthrusts = min_thrusts_ - tar_thrusts;
  qp_solver_.problem.b.head(z_rotors_.count()) = max_dthrusts;
  qp_solver_.problem.b.tail(z_rotors_.count()) = -min_dthrusts;

  const VectorXd dx = qp_solver_.solve();
  const auto dthrust = dx.tail(z_rotors_.count());
  return tar_thrusts + dthrust;
}

VectorXd Mixer::solve(
  const double& cur_voltage,
  const JntArray& cur_q,
  const Vector3d& cur_gyro_B,
  const Vector3d& tar_dgyro_B,
  const double& tar_thrusts_sum)
{
  // 均等に推力が分散されている状態を参照とする
  VectorXd tar_thrusts(z_rotors_.count());
  tar_thrusts.fill(tar_thrusts_sum / z_rotors_.count());

  return solve(cur_voltage, cur_q, cur_gyro_B, tar_dgyro_B, tar_thrusts);
}

void Mixer::configure(const MixerConfig& cfg)
{
  assert(cfg.dgyro_weight > 0);
  assert(cfg.thrust_weight > 0);

  cfg_ = cfg;
  updateQpWeight();
}

void Mixer::updateQpWeight()
{
  qp_solver_.problem.P.diagonal().head(3).fill(cfg_.dgyro_weight);
  qp_solver_.problem.P.diagonal().tail(z_rotors_.count()).fill(cfg_.thrust_weight);
}
}  // namespace tobas_mr_common
