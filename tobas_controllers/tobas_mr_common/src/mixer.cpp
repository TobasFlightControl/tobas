#include <tobas_std_tools/console.hpp>
#include <tobas_tools/constants.hpp>

#include "../include/tobas_mr_common/mixer.hpp"

using namespace std;
using namespace Eigen;
using namespace tobas_kdl;

namespace tobas_mr_common
{
Mixer::Mixer(const tobas::Drone& drone)
  : drone_(drone),
    fk_solver_(drone.tree()),
    jnt_axis_solver_(drone.tree()),
    inertia_solver_(drone.tree()),
    z_rotors_(drone, tobas::Axis::Z_POSITIVE)
{
  updateInternalDataStructures();
}

void Mixer::updateInternalDataStructures()
{
  fk_solver_.updateInternalDataStructures();
  jnt_axis_solver_.updateInternalDataStructures();
  inertia_solver_.updateInternalDataStructures();
  z_rotors_.updateInternalDataStructures();

  qp_.resize(3 + z_rotors_.count(), kEqualityConstSize, z_rotors_.count() * 2);
  qp_.setZero();

  // 機体の質量
  if (inertia_solver_.JntToCart(JntArray::Zero(drone_.tree().getNrOfJoints())) < 0)
    throw runtime_error("Inertia solver failed: " + inertia_solver_.errorMessage());

  // QPの決定変数のスケール
  constexpr double dgyro_scale = M_PI;
  qp_.x_scale.head(3).fill(dgyro_scale);
  const auto& mass = inertia_solver_.getInertia().getMass();
  const auto thrust_scale = mass * tobas::kGravity / z_rotors_.count();
  qp_.x_scale.tail(z_rotors_.count()).fill(thrust_scale);

  // QPPの定数部分
  qp_.problem.G.bottomRightCorner(1, z_rotors_.count()).fill(1);
  qp_.problem.A.topRightCorner(z_rotors_.count(), z_rotors_.count()).diagonal().fill(1);
  qp_.problem.A.bottomRightCorner(z_rotors_.count(), z_rotors_.count()).diagonal().fill(-1);

  // QPの重み
  updateQpWeight();

  U_.resize(NoChange, z_rotors_.count());
  max_thrusts_.resize(z_rotors_.count());
  min_thrusts_.resize(z_rotors_.count());
  last_thrusts_ = VectorXd::Zero(z_rotors_.count());
}

VectorXd Mixer::solve(
  const double& dt,
  const double& cur_voltage,
  const JntArray& cur_q,
  const Vector3d& cur_gyro_B,
  const Vector3d& cur_h_moment_B,
  const Vector3d& tar_dgyro_B,
  const VectorXd& tar_thrusts)
{
  assert(dt >= 0);
  assert(cur_voltage > 0);
  assert(static_cast<size_t>(tar_thrusts.size()) == z_rotors_.count());

  // 質量特性を計算
  if (inertia_solver_.JntToCart(cur_q) < 0)
    throw runtime_error("Inertia solver failed: " + inertia_solver_.errorMessage());
  const auto& inertia = inertia_solver_.getInertia();
  const auto B_Pos_B2G = inertia.getCOG();
  const auto I_B = inertia.getRotationalInertiaCoG();

  for (size_t i = 0; i < z_rotors_.count(); ++i)
  {
    // FKと回転軸を更新
    const auto& link_name = z_rotors_.linkName(i);
    if (fk_solver_.JntToCart(cur_q, link_name) < 0)
      throw runtime_error("Forward kinematics failed: " + fk_solver_.errorMessage());
    if (jnt_axis_solver_.JntToCart(cur_q, link_name) < 0)
      throw runtime_error("Joint axis solver failed: " + jnt_axis_solver_.errorMessage());

    const auto& B_Pos_B2P = fk_solver_.getFrame().p;
    const auto& axis_B = jnt_axis_solver_.getAxis();

    const auto& d = z_rotors_.direction(i);
    const auto& cm = z_rotors_.momentConstant(i);
    const auto B_Pos_G2P = B_Pos_B2P - B_Pos_B2G;
    U_.col(i) = ((d * cm) * axis_B - B_Pos_G2P * axis_B).data;
  }

  qp_.problem.G.topLeftCorner(3, 3) = I_B.data;
  qp_.problem.G.topRightCorner(3, z_rotors_.count()) = U_;

  const auto m_inertia = I_B.data * tar_dgyro_B;  // 慣性力によるモーメント
  const auto m_coriolis = cur_gyro_B.cross(I_B.data * cur_gyro_B);  // コリオリ力によるモーメント
  qp_.problem.h.head(3) = cur_h_moment_B - m_inertia - m_coriolis - U_ * tar_thrusts;
  // qp_.problem.h.head(3) = cur_h_moment_B - m_inertia - U_ * tar_thrusts;  // コリオリ力無視の場合

  updateThrustLimits(dt, cur_voltage, tar_thrusts.sum());
  const auto max_dthrusts = max_thrusts_ - tar_thrusts;
  const auto min_dthrusts = min_thrusts_ - tar_thrusts;
  qp_.problem.b.head(z_rotors_.count()) = max_dthrusts;
  qp_.problem.b.tail(z_rotors_.count()) = -min_dthrusts;

  const VectorXd dx = qp_.solve();
  const auto dthrust = dx.tail(z_rotors_.count());
  return last_thrusts_ = tar_thrusts + dthrust;
}

VectorXd Mixer::solve(
  const double& dt,
  const double& cur_voltage,
  const JntArray& cur_q,
  const Vector3d& cur_gyro_B,
  const Vector3d& cur_h_moment_B,
  const Vector3d& tar_dgyro_B,
  const double& tar_thrusts_sum)
{
  // 均等に推力が分散されている状態を参照とする
  VectorXd tar_thrusts = VectorXd::Constant(z_rotors_.count(), tar_thrusts_sum / z_rotors_.count());
  return solve(dt, cur_voltage, cur_q, cur_gyro_B, cur_h_moment_B, tar_dgyro_B, tar_thrusts);
}

void Mixer::configure(const MixerConfig& cfg)
{
  assert(cfg.dgyro_weight > 0);
  assert(cfg.thrust_weight > 0);
  assert(cfg.max_rot_acc > 0);

  cfg_ = cfg;
  updateQpWeight();
}

void Mixer::updateQpWeight()
{
  qp_.problem.P.diagonal().head(3).fill(cfg_.dgyro_weight);
  qp_.problem.P.diagonal().tail(z_rotors_.count()).fill(cfg_.thrust_weight);
}

void Mixer::updateThrustLimits(
  const double& dt,
  const double& cur_voltage,
  const double& thrusts_sum)
{
  tobas_std::Range<double> thrust_limit_1;
  tobas_std::Range<double> thrust_limit_2;

  for (size_t i = 0; i < z_rotors_.count(); ++i)
  {
    // ハードウェアによる制約
    thrust_limit_1.upper = z_rotors_.maxThrust(i, cur_voltage);
    thrust_limit_1.lower = z_rotors_.minThrust(i, cur_voltage);

    // 回転数の変化率による制約
    const auto max_drot = cfg_.max_rot_acc * dt;  // 回転数の変化量の最大値
    const auto& ct = z_rotors_.motorConstant(i);
    const auto& last_thrust = last_thrusts_(i);
    const auto max_dthrust = 2 * sqrt(ct * last_thrust) * max_drot + ct * tobas_std::sqr(max_drot);
    thrust_limit_2.upper = last_thrust + max_dthrust;
    thrust_limit_2.lower = last_thrust - max_dthrust;

    if (thrust_limit_1.isOverlapped(thrust_limit_2))
    {
      // 2つの制約の共通部分を求める
      const auto overlap = thrust_limit_1.overlappedArea(thrust_limit_2);
      max_thrusts_(i) = overlap.upper;
      min_thrusts_(i) = overlap.lower;
    }
    else
    {
      // 共通範囲が存在しない場合はハードウェア制約を優先
      max_thrusts_(i) = thrust_limit_1.upper;
      min_thrusts_(i) = thrust_limit_1.lower;
    }
  }

  // 合計推力の等式制約を満たせない場合は，不等式制約を取り除く
  if (thrusts_sum < min_thrusts_.sum() || max_thrusts_.sum() < thrusts_sum)
  {
    PRINT_ERROR("Target thrust sum is not within the limit.");
    max_thrusts_.fill(numeric_limits<double>::max());
    min_thrusts_.fill(0);
  }
}
}  // namespace tobas_mr_common
