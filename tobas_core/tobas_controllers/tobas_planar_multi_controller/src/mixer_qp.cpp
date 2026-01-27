#include "tobas_planar_multi_controller/mixer_qp.hpp"

#include <ranges>

#include <tobas_constants/constants.hpp>
#include <tobas_math/core.hpp>
#include <tobas_std_tools/universal_constants.hpp>

using namespace std;
using namespace Eigen;

namespace tobas
{
namespace planar_multicopter
{
QpMixer::QpMixer(const Drone& drone, const kdl::Tree& tree)
  : super(drone, tree), fk_solver_(tree), inertia_solver_(tree), stopwatch_(100)
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
  const kdl::Vector& cur_gyro_B,
  const kdl::Vector& tar_dgyro_B,
  const double& tar_thrusts_sum,
  const kdl::Vector& ext_torque_B)
{
  if (tar_thrusts_sum < 0.) {
    cerr << "Target thrust must be non-negative: " << tar_thrusts_sum << " < 0" << endl;
    return false;
  }

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
  for (const auto& [idx, pair] : views::enumerate(drone_.prop->rotors)) {
    const auto& rotor = pair.second;

    const auto& B_Pos_B2P = fk_solver_.getFrame(rotor->link_name).p;

    const auto& elem = tree_.getSegment(rotor->link_name)->second;
    const auto& B_Rot_Par = fk_solver_.getFrame(elem.parent->first).M;
    const auto axis_B = B_Rot_Par * elem.segment.joint().axis();

    const auto d = rotor->sign();
    const auto cm = rotor->momentConst();
    const auto B_Pos_G2P = B_Pos_B2P - B_Pos_B2G;
    G_.col(idx) = (B_Pos_G2P * axis_B - (d * cm) * axis_B).data;
  }

  // EoM行列等式の右辺
  h_ = (I_B * tar_dgyro_B + cur_gyro_B * (I_B * cur_gyro_B) - ext_torque_B).data;  // [Nm]

  // 重み
  const auto angular_scale = (I_B.trace() / 3) * kDGyroScale;                 // [Nm]
  const auto thrust_scale = mass * tbs::kGravity / drone_.prop->numRotors();  // [N]
  Q_.diagonal().fill(cfg_.base_weight / math::sqr(angular_scale));
  R_.diagonal().fill(cfg_.thrust_weight / math::sqr(thrust_scale));

  // コスト関数
  qp_.problem.P = G_.transpose() * Q_ * G_;
  qp_.problem.P.diagonal() += R_.diagonal();
  qp_.problem.q = -G_.transpose() * Q_ * h_;

  // 不等式制約
  // 同時に合計推力の範囲を計算
  double max_thrust_sum = 0.;
  double min_thrust_sum = 0.;
  for (const auto& [idx, pair] : views::enumerate(drone_.prop->rotors)) {
    const auto& rotor = pair.second;

    double max_thrust, min_thrust;
    if (rotor_alive_.at(rotor->link_name)) {
      max_thrust = drone_.prop->maxThrust(rotor->link_name);
      min_thrust = drone_.prop->minThrust(rotor->link_name);
    }
    else {
      max_thrust = 0.;
      min_thrust = 0.;
    }

    qp_.problem.b(idx) = max_thrust;
    qp_.problem.b(drone_.prop->numRotors() + idx) = -min_thrust;
    max_thrust_sum += max_thrust;
    min_thrust_sum += min_thrust;
  }

  // 推力を出せない状態の場合は終了
  if (max_thrust_sum == 0.) {
    cerr << "The vehicle cannot generate thrust." << endl;
    return false;
  }

  // 等式制約
  // 不等式制約と競合しないようにクランプ
  static constexpr double kThrustClampMargin = 1e-3;  // [N]
  qp_.problem.h(0) = clamp(tar_thrusts_sum, min_thrust_sum + kThrustClampMargin, max_thrust_sum - kThrustClampMargin);

  // QPPを解く
  // TODO: 正則化項を入れると必ず解のシフトが発生するため，階層QPを使うか，Gのランクによって分岐
  // stopwatch_.start();
  if (!qp_.solve()) {
    cerr << "QP failed: " << qp_.errorMessage() << endl;
    return false;
  }
  // stopwatch_.stop();

  return true;
}

double QpMixer::getThrust(size_t idx) const
{
  return thrustDeadband(qp_.solution()(idx));
}

bool QpMixer::setBaseWeight(double p)
{
  if (p <= 0.) {
    cerr << "Base weight must be positive." << endl;
    return false;
  }

  cfg_.base_weight = p;
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

void QpMixer::resizeAndFill()
{
  const auto nr = drone_.prop->numRotors();

  qp_.resize(nr, 1, nr * 2);
  qp_.setZero();

  // QPの決定変数のスケール．推力のみなので統一してよい．
  qp_.x_scale.setOnes();

  // QPPの定数部分
  qp_.problem.G.fill(1);
  qp_.problem.A.topRows(nr).diagonal().fill(1);
  qp_.problem.A.bottomRows(nr).diagonal().fill(-1);

  R_.resize(nr);
  G_.resize(NoChange, nr);
}
}  // namespace planar_multicopter
}  // namespace tobas
