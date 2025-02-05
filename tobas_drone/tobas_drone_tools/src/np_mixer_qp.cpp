#include <ranges>

#include <tobas_std_tools/check.hpp>
#include <tobas_std_tools/universal_constants.hpp>
#include <tobas_constants/constants.hpp>

#include "../include/tobas_drone_tools/np_mixer_qp.hpp"

using namespace std;
using namespace Eigen;

namespace tobas
{
NonPlanarMixer_QP::NonPlanarMixer_QP(const Drone& drone, const kdl::Tree& tree)
  : drone_(drone), tree_(tree), fk_solver_(tree), inertia_solver_(tree)
{
  resizeAndFill();
  updateWeight();
}

bool NonPlanarMixer_QP::updateInternalDataStructures()
{
  if (!fk_solver_.updateInternalDataStructures())
    return false;
  if (!inertia_solver_.updateInternalDataStructures())
    return false;

  resizeAndFill();
  updateWeight();

  return true;
}

bool NonPlanarMixer_QP::solve(
  const double& cur_voltage,
  const kdl::JntArray& cur_q,
  const kdl::Rotation& cur_rot,
  const kdl::Vector& cur_gyro_B,
  const kdl::Vector& tar_acc_W,
  const kdl::Vector& tar_dgyro_B,
  const kdl::Vector& ext_force_W,
  const kdl::Vector& ext_torque_B)
{
  assert(cur_voltage > 0);

  // 順運動学を計算
  if (fk_solver_.JntToCart(cur_q) < 0)
  {
    cerr << "Forward kinematics failed: " << fk_solver_.errorMessage() << endl;
    return false;
  }

  // 質量特性を計算
  if (inertia_solver_.JntToCart(cur_q) < 0)
  {
    cerr << "Inertia solver failed: " << inertia_solver_.errorMessage() << endl;
    return false;
  }
  const auto& inertia = inertia_solver_.getInertia();
  const auto B_Pos_B2G = inertia.getCOG();
  const auto I_B = inertia.getRotationalInertiaCoG();
  const auto& mass = inertia.getMass();

  // EoM行列等式の左辺
  for (const auto& [idx, rotor_it] : views::enumerate(drone_.rotors))
  {
    const auto& rotor = rotor_it.second;

    // 回転軸を求める
    const auto elem = tree_.getSegment(rotor.link_name)->second;
    const auto& B_Rot_Par = fk_solver_.getFrame(elem.parent->first).M;
    const auto axis_B = B_Rot_Par * elem.segment.joint().axis();

    // 並進
    G_.block<3, 1>(0, idx) = axis_B.data;

    // 回転
    const auto d = rotor.sign();
    const auto& cm = rotor.moment_constant;
    const auto& B_Pos_B2P = fk_solver_.getFrame(rotor.link_name).p;
    const auto B_Pos_G2P = B_Pos_B2P - B_Pos_B2G;
    G_.block<3, 1>(3, idx) = (B_Pos_G2P * axis_B - (d * cm) * axis_B).data;
  }

  // EoM行列等式の右辺
  const kdl::Vector grav_W(0, 0, -tobas_std::kGravity);
  const auto trans_right = mass * cur_rot.inverse(tar_acc_W - grav_W) - ext_force_W;
  const auto rot_right = I_B * tar_dgyro_B + cur_gyro_B * (I_B * cur_gyro_B) - ext_torque_B;
  h_.head<3>() = trans_right.data;
  h_.tail<3>() = rot_right.data;

  // コスト関数
  qp_.problem.P = G_.transpose() * Q_ * G_;
  qp_.problem.P.diagonal() += R_.diagonal();
  qp_.problem.q = -G_.transpose() * Q_ * h_;

  // 不等式制約
  for (const auto& [idx, rotor_it] : views::enumerate(drone_.rotors))
  {
    const auto& rotor = rotor_it.second;
    qp_.problem.b(idx) = rotor.maxThrust(cur_voltage);
    qp_.problem.b(drone_.numRotors() + idx) = -rotor.minThrust();
  }

  // QPPを解く
  // TODO: 正則化項を入れると必ず解のシフトが発生するため，階層QPを使うか，Gのランクによって分岐
  if (!qp_.solve())
  {
    cerr << "QP failed: " << qp_.errorMessage() << endl;
    return false;
  }

  return true;
}

const VectorXd& NonPlanarMixer_QP::getThrusts() const
{
  return qp_.solution();
}

bool NonPlanarMixer_QP::setLinearWeight(double p)
{
  if (p <= 0.)
  {
    cerr << "Linear weight must be positive." << endl;
    return false;
  }

  linear_weight_ = p;
  updateWeight();
  return true;
}

bool NonPlanarMixer_QP::setAngularWeight(double p)
{
  if (p <= 0.)
  {
    cerr << "Angular weight must be positive." << endl;
    return false;
  }

  angular_weight_ = p;
  updateWeight();
  return true;
}

bool NonPlanarMixer_QP::setThrustWeight(double p)
{
  if (p <= 0.)
  {
    cerr << "Thrust weight must be positive." << endl;
    return false;
  }

  thrust_weight_ = p;
  updateWeight();
  return true;
}

void NonPlanarMixer_QP::resizeAndFill()
{
  const auto nr = drone_.numRotors();

  qp_.resize(nr, 0, nr * 2);
  qp_.setZero();

  // QPの決定変数のスケール．推力のみなので統一してよい．
  qp_.x_scale.setOnes();

  // QPPの定数部分
  qp_.problem.A.topRows(nr).diagonal().fill(1);
  qp_.problem.A.bottomRows(nr).diagonal().fill(-1);

  R_.resize(nr);
  G_.resize(NoChange, nr);
}

void NonPlanarMixer_QP::updateWeight()
{
  if (drone_.numRotors() == 0)
    return;

  if (inertia_solver_.JntToCart(kdl::JntArray::Zero(tree_.getNrOfJoints())) < 0)
    throw runtime_error("Inertia solver failed: " + inertia_solver_.errorMessage());
  const auto& inertia = inertia_solver_.getInertia();
  const auto& mass = inertia.getMass();
  const auto& I = inertia.getRotationalInertia();

  const auto linear_scale = mass * kAccelScale;                               // [N]
  const auto angular_scale = (I.trace() / 3) * kDGyroScale;                   // [Nm]
  const auto thrust_scale = mass * tobas_std::kGravity / drone_.numRotors();  // [N]

  Q_.diagonal().head<3>().fill(linear_weight_ / math::sqr(linear_scale));
  Q_.diagonal().tail<3>().fill(angular_weight_ / math::sqr(angular_scale));
  R_.diagonal().fill(thrust_weight_ / math::sqr(thrust_scale));
}
}  // namespace tobas
