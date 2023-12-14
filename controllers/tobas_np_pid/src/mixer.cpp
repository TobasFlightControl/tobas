#include <dh_std_tools/console.hpp>
#include <dh_std_tools/check.hpp>
#include <dh_std_tools/console.hpp>

#include <tobas_tools/constants.hpp>

#include "../include/tobas_np_pid/mixer.hpp"

#define ACC_SCALE tobas::kGravity
#define DGYRO_SCALE M_PI

using namespace std;
using namespace Eigen;
using namespace KDL;

namespace tobas_np_pid
{
Mixer::Mixer(const tobas::Drone& drone)
  : drone_(drone),
    fk_solver_(drone.tree()),
    jnt_axis_solver_(drone.tree()),
    inertia_solver_(drone.tree())
{
  DH_DEBUG("Mixer::Mixer");

  updateInternalDataStructures();
}

void Mixer::updateInternalDataStructures()
{
  DH_DEBUG("Mixer::updateInternalDataStructures");

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
  cog2prop_B_.resize(drone_.numRotors());
  axis_B_.resize(drone_.numRotors());
}

VectorXd Mixer::solve(
  const double& cur_voltage,
  const JntArray& cur_q,
  const Euler& cur_rpy,
  const Vector& cur_gyro_B,
  const Vector& tar_acc_W,
  const Vector& tar_dgyro_B)
{
  DH_DEBUG_ONCE("Mixer::solve");

  assert(cur_voltage > 0);

  // 質量特性を計算
  if (inertia_solver_.JntToCart(cur_q) < 0)
    throw runtime_error("Inertia solver failed: " + inertia_solver_.errorMessage());
  const auto& inertia = inertia_solver_.getInertia();
  const auto B_Pos_B2G = inertia.getCOG();
  const auto I_B = inertia.refPoint(B_Pos_B2G).getRotationalInertia();
  const auto& mass = inertia.getMass();

  // プロペラの位置と回転軸を更新
  for (size_t i = 0; i < drone_.numRotors(); ++i)
  {
    const auto& link_name = drone_.rotorConfig(i).link_name;

    if (fk_solver_.JntToCart(cur_q, link_name) < 0)
      throw runtime_error("Forward kinematics failed: " + fk_solver_.errorMessage());
    cog2prop_B_[i] = fk_solver_.getFrame().p - B_Pos_B2G;

    if (jnt_axis_solver_.JntToCart(cur_q, link_name) < 0)
      throw runtime_error("Joint axis solver failed: " + jnt_axis_solver_.errorMessage());
    axis_B_[i] = jnt_axis_solver_.getAxis();
  }

  // EoM行列等式の左辺
  for (size_t i = 0; i < drone_.numRotors(); ++i)
  {
    // 並進
    G_.block<3, 1>(0, i) = axis_B_[i].data;

    // 回転
    const auto& d = drone_.rotorConfig(i).direction;
    const auto& cm = drone_.rotorConfig(i).moment_constant;
    G_.block<3, 1>(3, i) = (cog2prop_B_[i] * axis_B_[i] - (d * cm) * axis_B_[i]).data;
  }

  // EoM行列等式の右辺
  // TODO: H-forceを考慮
  const auto trans_right = mass * cur_rpy.toRotation().inverse(tar_acc_W - tobas::kWorldGravity);
  const auto rot_right = I_B * tar_dgyro_B + cur_gyro_B * (I_B * cur_gyro_B);
  h_.head<3>() = trans_right.data;
  h_.tail<3>() = rot_right.data;

  // コスト関数
  qp_.problem.P = G_.transpose() * Q_ * G_;
  qp_.problem.P.diagonal() += R_.diagonal();
  qp_.problem.q = -h_.transpose() * Q_ * G_;

  // 不等式制約
  const auto min_voltage = cur_voltage * tobas::kArmThrottle;
  for (size_t i = 0; i < drone_.numRotors(); ++i)
  {
    qp_.problem.b(i) = drone_.thrustFromVoltage(i, cur_voltage);
    qp_.problem.b(drone_.numRotors() + i) = -drone_.thrustFromVoltage(i, min_voltage);
  }

  // QPPを解く
  // TODO: 正則化項を入れると必ず解のシフトが発生するため，階層QPを使うか，Gのランクによって分岐
  return qp_.solve();
}

void Mixer::configure(const MixerConfig& cfg)
{
  DH_DEBUG("Mixer::configure");

  if (!drone_.isLoaded())
    throw runtime_error("Drone is not loaded yet.");

  CHECK(cfg.linear_weight > 0);
  CHECK(cfg.angular_weight > 0);

  if (inertia_solver_.JntToCart(JntArray::Zero(drone_.tree().getNrOfJoints())) < 0)
    throw runtime_error("Inertia solver failed: " + inertia_solver_.errorMessage());
  const auto& inertia = inertia_solver_.getInertia();
  const auto& mass = inertia.getMass();
  const auto& I = inertia.getRotationalInertia();  // トレースがほしいだけ

  const auto linear_scale = mass * ACC_SCALE;
  const auto angular_scale = I.trace() / 3 * DGYRO_SCALE;
  const auto thrust_scale = mass * tobas::kGravity / drone_.numRotors();

  Q_.diagonal().head<3>().fill(cfg.linear_weight / sqr(linear_scale));
  Q_.diagonal().tail<3>().fill(cfg.angular_weight / sqr(angular_scale));
  R_.diagonal().fill(exp10(cfg.thrust_weight_log10) / sqr(thrust_scale));
}
}  // namespace tobas_np_pid
