#include <tobas_std_tools/check.hpp>
#include <tobas_tools/constants.hpp>

#include "../include/tobas_np_pid/mixer.hpp"

using namespace std;
using namespace Eigen;

namespace tobas_np_pid
{
Mixer::Mixer(const tobas::Drone& drone)
  : drone_(drone), fk_solver_(drone.tree()), jnt_axis_solver_(drone.tree()), inertia_solver_(drone.tree())
{
  updateInternalDataStructures();
}

void Mixer::updateInternalDataStructures()
{
  fk_solver_.updateInternalDataStructures();
  jnt_axis_solver_.updateInternalDataStructures();
  inertia_solver_.updateInternalDataStructures();

  qp_.resize(drone_.numRotors(), 0, drone_.numRotors() * 2);
  qp_.setZero();

  // QPの決定変数のスケール．推力のみなので統一してよい．
  qp_.x_scale.setOnes();

  // QPPの定数部分
  qp_.problem.A.topRows(drone_.numRotors()).diagonal().fill(1);
  qp_.problem.A.bottomRows(drone_.numRotors()).diagonal().fill(-1);

  R_.resize(drone_.numRotors());
  G_.resize(NoChange, drone_.numRotors());
}

VectorXd Mixer::solve(
  const double& cur_voltage,
  const kdl::JntArray& cur_q,
  const kdl::Rotation& cur_rot,
  const kdl::Vector& cur_gyro_B,
  const kdl::Vector& tar_acc_W,
  const kdl::Vector& tar_dgyro_B)
{
  assert(cur_voltage > 0);

  // 質量特性を計算
  if (inertia_solver_.JntToCart(cur_q) < 0)
    throw runtime_error("Inertia solver failed: " + inertia_solver_.errorMessage());
  const auto& inertia = inertia_solver_.getInertia();
  const auto B_Pos_B2G = inertia.getCOG();
  const auto I_B = inertia.getRotationalInertiaCoG();
  const auto& mass = inertia.getMass();

  // EoM行列等式の左辺
  for (size_t i = 0; i < drone_.numRotors(); ++i)
  {
    // FKと回転軸を更新
    const auto& link_name = drone_.rotorConfig(i).link_name;
    if (fk_solver_.JntToCart(cur_q, link_name) < 0)
      throw runtime_error("Forward kinematics failed: " + fk_solver_.errorMessage());
    if (jnt_axis_solver_.JntToCart(cur_q, link_name) < 0)
      throw runtime_error("Joint axis solver failed: " + jnt_axis_solver_.errorMessage());

    const auto& B_Pos_B2P = fk_solver_.getFrame().p;
    const auto& axis_B = jnt_axis_solver_.getAxis();

    // 並進
    G_.block<3, 1>(0, i) = axis_B.data;

    // 回転
    const auto& d = drone_.rotorConfig(i).direction.value;
    const auto& cm = drone_.rotorConfig(i).moment_constant;
    const auto B_Pos_G2P = B_Pos_B2P - B_Pos_B2G;
    G_.block<3, 1>(3, i) = (B_Pos_G2P * axis_B - (d * cm) * axis_B).data;
  }

  // EoM行列等式の右辺
  // TODO: H-forceを考慮
  const kdl::Vector grav_W(0, 0, -tobas::kGravity);
  const auto trans_right = mass * cur_rot.inverse(tar_acc_W - grav_W);
  const auto rot_right = I_B * tar_dgyro_B + cur_gyro_B * (I_B * cur_gyro_B);
  h_.head<3>() = trans_right.data;
  h_.tail<3>() = rot_right.data;

  // コスト関数
  qp_.problem.P = G_.transpose() * Q_ * G_;
  qp_.problem.P.diagonal() += R_.diagonal();
  qp_.problem.q = -h_.transpose() * Q_ * G_;

  // 不等式制約
  for (size_t i = 0; i < drone_.numRotors(); ++i)
  {
    qp_.problem.b(i) = drone_.maxThrust(i, cur_voltage);
    qp_.problem.b(drone_.numRotors() + i) = -drone_.minThrust(i, cur_voltage);
  }

  // QPPを解く
  // TODO: 正則化項を入れると必ず解のシフトが発生するため，階層QPを使うか，Gのランクによって分岐
  if (!qp_.solve())
    throw runtime_error("QP failed: " + qp_.errorMessage());

  return qp_.solution();
}

void Mixer::configure(const MixerConfig& cfg)
{
  if (!drone_.isLoaded())
    throw runtime_error("Drone is not loaded yet.");

  TOBAS_CHECK(cfg.linear_weight > 0);
  TOBAS_CHECK(cfg.angular_weight > 0);

  if (inertia_solver_.JntToCart(kdl::JntArray::Zero(drone_.tree().getNrOfJoints())) < 0)
    throw runtime_error("Inertia solver failed: " + inertia_solver_.errorMessage());
  const auto& inertia = inertia_solver_.getInertia();
  const auto& mass = inertia.getMass();
  const auto& I = inertia.getRotationalInertia();  // トレースがほしいだけ

  const auto linear_scale = mass * tobas::kGravity;
  const auto angular_scale = I.trace() / 3 * M_PI;
  const auto thrust_scale = mass * tobas::kGravity / drone_.numRotors();

  Q_.diagonal().head<3>().fill(cfg.linear_weight / math::sqr(linear_scale));
  Q_.diagonal().tail<3>().fill(cfg.angular_weight / math::sqr(angular_scale));
  R_.diagonal().fill(exp10(cfg.thrust_weight_log10) / math::sqr(thrust_scale));
}
}  // namespace tobas_np_pid
