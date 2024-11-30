#include <tobas_std_tools/console.hpp>
#include <tobas_std_tools/universal_constants.hpp>
#include <tobas_constants/constants.hpp>

#include "../include/tobas_drone_tools/mr_mixer.hpp"

using namespace std;
using namespace Eigen;

namespace tobas
{
MultiRotorMixer::MultiRotorMixer(const Drone& drone, const kdl::Tree& tree)
  : drone_(drone),
    tree_(tree),
    fk_solver_(tree),
    jnt_axis_solver_(tree),
    inertia_solver_(tree),
    z_rotors_(drone, Z_POSITIVE)
{
  updateInternalDataStructures();
}

void MultiRotorMixer::updateInternalDataStructures()
{
  fk_solver_.updateInternalDataStructures();
  jnt_axis_solver_.updateInternalDataStructures();
  inertia_solver_.updateInternalDataStructures();
  z_rotors_.updateInternalDataStructures();

  qp_.resize(z_rotors_.count(), 1, z_rotors_.count() * 2);
  qp_.setZero();

  // QPの決定変数のスケール．推力のみなので統一してよい．
  qp_.x_scale.setOnes();

  // QPPの定数部分
  qp_.problem.G.fill(1);
  qp_.problem.A.topRows(z_rotors_.count()).diagonal().fill(1);
  qp_.problem.A.bottomRows(z_rotors_.count()).diagonal().fill(-1);

  R_.resize(z_rotors_.count());
  G_.resize(NoChange, z_rotors_.count());

  // 重みを更新
  updateWeight();
}

bool MultiRotorMixer::solve(
  const double& cur_voltage,
  const kdl::JntArray& cur_q,
  const kdl::Vector& cur_gyro_B,
  const kdl::Vector& tar_dgyro_B,
  const double& tar_thrusts_sum)
{
  assert(cur_voltage > 0);
  assert(tar_thrusts_sum > 0);

  // 質量特性を計算
  if (inertia_solver_.JntToCart(cur_q) < 0)
  {
    cerr << "Inertia solver failed: " << inertia_solver_.errorMessage() << endl;
    return false;
  }
  const auto& inertia = inertia_solver_.getInertia();
  const auto B_Pos_B2G = inertia.getCOG();
  const auto I_B = inertia.getRotationalInertiaCoG();

  // EoM行列等式の左辺
  for (size_t i = 0; i < z_rotors_.count(); ++i)
  {
    const auto& rotor = z_rotors_.rotor(i);

    // FKと回転軸を更新
    if (fk_solver_.JntToCart(cur_q, rotor.link_name) < 0)
    {
      cerr << "Forward kinematics failed: " << fk_solver_.errorMessage() << endl;
      return false;
    }
    if (jnt_axis_solver_.JntToCart(cur_q, rotor.link_name) < 0)
    {
      cerr << "Joint axis solver failed: " << jnt_axis_solver_.errorMessage() << endl;
      return false;
    }

    const auto& B_Pos_B2P = fk_solver_.getFrame().p;
    const auto& axis_B = jnt_axis_solver_.getAxis();

    const auto d = rotor.sign();
    const auto& cm = rotor.moment_constant;
    const auto B_Pos_G2P = B_Pos_B2P - B_Pos_B2G;
    G_.col(i) = (B_Pos_G2P * axis_B - (d * cm) * axis_B).data;
  }

  // EoM行列等式の右辺
  const auto right = I_B * tar_dgyro_B + cur_gyro_B * (I_B * cur_gyro_B);
  h_ = right.data;

  // コスト関数
  qp_.problem.P = G_.transpose() * Q_ * G_;
  qp_.problem.P.diagonal() += R_.diagonal();
  qp_.problem.q = -G_.transpose() * Q_ * h_;

  // 等式制約
  qp_.problem.h(0) = tar_thrusts_sum;

  // 不等式制約
  for (size_t i = 0; i < z_rotors_.count(); ++i)
  {
    const auto& rotor = z_rotors_.rotor(i);
    qp_.problem.b(i) = rotor.maxThrust(cur_voltage);
    qp_.problem.b(z_rotors_.count() + i) = -rotor.minThrust(cur_voltage);
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

const VectorXd& MultiRotorMixer::getThrusts() const
{
  return qp_.solution();
}

bool MultiRotorMixer::setBaseWeight(double p)
{
  if (p <= 0.)
  {
    cerr << "Base weight must be positive." << endl;
    return false;
  }

  base_weight_ = p;
  updateWeight();
  return true;
}

bool MultiRotorMixer::setThrustWeight(double p)
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

void MultiRotorMixer::updateWeight()
{
  if (z_rotors_.count() == 0)
    return;

  if (inertia_solver_.JntToCart(kdl::JntArray::Zero(tree_.getNrOfJoints())) < 0)
    throw runtime_error("Inertia solver failed: " + inertia_solver_.errorMessage());
  const auto& inertia = inertia_solver_.getInertia();
  const auto& mass = inertia.getMass();
  const auto& I = inertia.getRotationalInertia();

  const auto angular_scale = (I.trace() / 3) * kDGyroScale;                  // [Nm]
  const auto thrust_scale = mass * tobas_std::kGravity / z_rotors_.count();  // [N]

  Q_.diagonal().fill(base_weight_ / math::sqr(angular_scale));
  R_.diagonal().fill(thrust_weight_ / math::sqr(thrust_scale));
}
}  // namespace tobas
