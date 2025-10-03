#include "tobas_nonplanar_multi_controller/mixer_qp.hpp"

#include <ranges>

#include <tobas_constants/constants.hpp>
#include <tobas_eigen_tools/operators.hpp>
#include <tobas_math/core.hpp>
#include <tobas_std_tools/universal_constants.hpp>

using namespace std;
using namespace Eigen;

namespace tobas
{
namespace nonplanar_multicopter
{
QpMixer::QpMixer(const Drone& drone, const kdl::Tree& tree)
  : super(drone, tree), fk_solver_(tree), inertia_solver_(tree)
{
}

bool QpMixer::updateInternalDataStructures()
{
  if (!super::updateInternalDataStructures()) {
    return false;
  }

  if (!fk_solver_.updateInternalDataStructures()) {
    return false;
  }
  if (!inertia_solver_.updateInternalDataStructures()) {
    return false;
  }

  resizeAndFill();

  return true;
}

bool QpMixer::solve(
  const kdl::JntArray& cur_q,
  const kdl::Rotation& cur_rot,
  const kdl::Vector& cur_gyro_B,
  const kdl::Vector& tar_acc_W,
  const kdl::Vector& tar_dgyro_B,
  const kdl::Vector& ext_force_W,
  const kdl::Vector& ext_torque_B)
{
  // 順運動学を計算
  if (fk_solver_.jntToCart(cur_q) < 0) {
    cerr << "Forward kinematics failed: " << fk_solver_.errorMessage() << endl;
    return false;
  }

  // 質量特性を計算
  if (inertia_solver_.jntToCart(cur_q) < 0) {
    cerr << "Inertia solver failed: " << inertia_solver_.errorMessage() << endl;
    return false;
  }
  const auto& inertia = inertia_solver_.getInertia();
  const auto& mass = inertia.getMass();
  const auto B_Pos_B2G = inertia.getCOG();
  const auto I_B = inertia.getRotationalInertiaCoG();

  // EoM行列等式の左辺
  for (const auto& [idx, rotor_it] : views::enumerate(drone_.prop->rotors)) {
    const auto& rotor = rotor_it.second;

    // 回転軸を求める
    const auto& elem = tree_.getSegment(rotor->link_name)->second;
    const auto& B_Rot_Par = fk_solver_.getFrame(elem.parent->first).M;
    const auto axis_B = B_Rot_Par * elem.segment.joint().axis();

    // 並進
    G_.block<3, 1>(0, idx) = axis_B.data;

    // 回転
    const auto d = rotor->sign();
    const auto cm = rotor->momentConst();
    const auto& B_Pos_B2P = fk_solver_.getFrame(rotor->link_name).p;
    const auto B_Pos_G2P = B_Pos_B2P - B_Pos_B2G;
    G_.block<3, 1>(3, idx) = (B_Pos_G2P * axis_B - (d * cm) * axis_B).data;
  }

  // 並進EoMの右辺
  const kdl::Vector grav_W(0, 0, -tobas_std::kGravity);
  auto eom_trans_right_W = mass * (tar_acc_W - grav_W) - ext_force_W;  // [N]
  eom_trans_right_W.z(max(eom_trans_right_W.z(), 0.));  // 鉛直下方向に推力を出さないよう制限
  h_.head<3>() = cur_rot.inverse(eom_trans_right_W).data;

  // 回転EoMの右辺
  const auto eom_rot_right_B = I_B * tar_dgyro_B + cur_gyro_B * (I_B * cur_gyro_B) - ext_torque_B;  // [Nm]
  h_.tail<3>() = eom_rot_right_B.data;

  // 重み
  const auto linear_scale = mass * kAccelScale;                                     // [N]
  const auto angular_scale = (I_B.trace() / 3) * kDGyroScale;                       // [Nm]
  const auto thrust_scale = mass * tobas_std::kGravity / drone_.prop->numRotors();  // [N]
  Q_.diagonal().head<3>().fill(cfg_.linear_weight / math::sqr(linear_scale));
  Q_.diagonal().tail<3>().fill(cfg_.angular_weight / math::sqr(angular_scale));
  R_.diagonal().fill(cfg_.thrust_weight / math::sqr(thrust_scale));
  S_.diagonal().fill(cfg_.delta_thrust_weight / math::sqr(thrust_scale));

  // 推力を決定変数としたときの目的関数
  const MatrixXd P = G_.transpose() * Q_ * G_ + R_;
  const VectorXd q = -G_.transpose() * Q_ * h_;

  // 推力を決定変数としたときの不等式制約
  for (const auto& [idx, rotor_it] : views::enumerate(drone_.prop->rotors)) {
    const auto& rotor = rotor_it.second;
    if (rotor_alive_.at(rotor->link_name)) {
      b_(idx) = drone_.prop->maxThrust(rotor->link_name);
      b_(drone_.prop->numRotors() + idx) = -drone_.prop->minThrust(rotor->link_name);
    }
    else {
      b_(idx) = 0.;
      b_(drone_.prop->numRotors() + idx) = 0.;
    }
  }

  // QPPの決定変数を推力からその変化量に変換 (memo: 3-40)
  qp_.problem.P = P + S_;
  qp_.problem.q = q + P * x_prev_;
  qp_.problem.A = A_;
  qp_.problem.b = b_ - A_ * x_prev_;

  // QPPを解く
  // TODO: 正則化項を入れると必ず解のシフトが発生するため，階層QPを使うか，Gのランクによって分岐
  if (!qp_.solve()) {
    cerr << "QP failed: " << qp_.errorMessage() << endl;
    return false;
  }

  // 解を保存
  const auto& x_delta = qp_.solution();
  x_prev_ += x_delta;

  return true;
}

const Eigen::VectorXd& QpMixer::getThrusts() const
{
  return x_prev_;
}

double QpMixer::getThrust(size_t idx) const
{
  return thrustDeadband(x_prev_(idx));
}

bool QpMixer::setLinearWeight(double p)
{
  if (p <= 0.) {
    cerr << "Linear weight must be positive." << endl;
    return false;
  }

  cfg_.linear_weight = p;
  return true;
}

bool QpMixer::setAngularWeight(double p)
{
  if (p <= 0.) {
    cerr << "Angular weight must be positive." << endl;
    return false;
  }

  cfg_.angular_weight = p;
  return true;
}

bool QpMixer::setThrustWeight(double p)
{
  if (p <= 0.) {
    cerr << "Thrust weight must be positive." << endl;
    return false;
  }

  cfg_.thrust_weight = p;
  return true;
}

bool QpMixer::setDeltaThrustWeight(double p)
{
  if (p <= 0.) {
    cerr << "Delta thrust weight must be positive." << endl;
    return false;
  }

  cfg_.delta_thrust_weight = p;
  return true;
}

void QpMixer::resizeAndFill()
{
  const auto var_size = drone_.prop->numRotors();
  const auto ineq_size = var_size * 2;

  qp_.resize(var_size, 0, ineq_size);
  qp_.x_scale.setOnes();  // QPの決定変数は推力のみなのでスケールは統一してよい

  R_.resize(var_size);
  S_.resize(var_size);

  G_.resize(NoChange, var_size);

  A_ = MatrixXd::Zero(ineq_size, var_size);
  A_.topRows(var_size).diagonal().fill(1);
  A_.bottomRows(var_size).diagonal().fill(-1);

  b_.resize(ineq_size);

  x_prev_ = VectorXd::Zero(var_size);
}
}  // namespace nonplanar_multicopter
}  // namespace tobas
