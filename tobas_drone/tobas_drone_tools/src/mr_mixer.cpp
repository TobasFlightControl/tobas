#include <tobas_std_tools/console.hpp>
#include <tobas_std_tools/universal_constants.hpp>
#include <tobas_constants/constants.hpp>

#include "../include/tobas_drone_tools/mr_mixer.hpp"

using namespace std;
using namespace Eigen;

namespace tobas
{
Mixer::Mixer(const Drone& drone, const kdl::Tree& tree)
  : drone_(drone),
    tree_(tree),
    fk_solver_(tree),
    jnt_axis_solver_(tree),
    inertia_solver_(tree),
    z_rotors_(drone, Z_POSITIVE)
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
  if (inertia_solver_.JntToCart(kdl::JntArray::Zero(tree_.getNrOfJoints())) < 0)
    throw runtime_error("Inertia solver failed: " + inertia_solver_.errorMessage());

  // QPの決定変数のスケール
  constexpr double dgyro_scale = M_PI;
  qp_.x_scale.head(3).fill(dgyro_scale);
  const auto& mass = inertia_solver_.getInertia().getMass();
  const auto thrust_scale = mass * tobas_std::kGravity / z_rotors_.count();
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
  const double& cur_voltage,
  const kdl::JntArray& cur_q,
  const Vector3d& cur_gyro_B,
  const Vector3d& cur_h_moment_B,
  const Vector3d& tar_dgyro_B,
  const VectorXd& tar_thrusts)
{
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
    const auto& rotor = z_rotors_.rotor(i);

    // FKと回転軸を更新
    if (fk_solver_.JntToCart(cur_q, rotor.link_name) < 0)
      throw runtime_error("Forward kinematics failed: " + fk_solver_.errorMessage());
    if (jnt_axis_solver_.JntToCart(cur_q, rotor.link_name) < 0)
      throw runtime_error("Joint axis solver failed: " + jnt_axis_solver_.errorMessage());

    const auto& B_Pos_B2P = fk_solver_.getFrame().p;
    const auto& axis_B = jnt_axis_solver_.getAxis();

    const auto d = rotor.sign();
    const auto& cm = rotor.moment_constant;
    const auto B_Pos_G2P = B_Pos_B2P - B_Pos_B2G;
    U_.col(i) = ((d * cm) * axis_B - B_Pos_G2P * axis_B).data;
  }

  qp_.problem.G.topLeftCorner(3, 3) = I_B.data;
  qp_.problem.G.topRightCorner(3, z_rotors_.count()) = U_;

  const auto m_inertia = I_B.data * tar_dgyro_B;                    // 慣性力によるモーメント
  const auto m_coriolis = cur_gyro_B.cross(I_B.data * cur_gyro_B);  // コリオリ力によるモーメント
  qp_.problem.h.head(3) = cur_h_moment_B - m_inertia - m_coriolis - U_ * tar_thrusts;
  // qp_.problem.h.head(3) = cur_h_moment_B - m_inertia - U_ * tar_thrusts;  // コリオリ力無視の場合

  updateThrustLimits(cur_voltage, tar_thrusts.sum());
  const auto max_dthrusts = max_thrusts_ - tar_thrusts;
  const auto min_dthrusts = min_thrusts_ - tar_thrusts;
  qp_.problem.b.head(z_rotors_.count()) = max_dthrusts;
  qp_.problem.b.tail(z_rotors_.count()) = -min_dthrusts;

  if (!qp_.solve())
    throw runtime_error("QP failed: " + qp_.errorMessage());

  const auto& dx = qp_.solution();
  const auto dthrust = dx.tail(z_rotors_.count());
  return last_thrusts_ = tar_thrusts + dthrust;
}

VectorXd Mixer::solve(
  const double& cur_voltage,
  const kdl::JntArray& cur_q,
  const Vector3d& cur_gyro_B,
  const Vector3d& cur_h_moment_B,
  const Vector3d& tar_dgyro_B,
  const double& tar_thrusts_sum)
{
  // 均等に推力が分散されている状態を参照とする
  VectorXd tar_thrusts = VectorXd::Constant(z_rotors_.count(), tar_thrusts_sum / z_rotors_.count());
  return solve(cur_voltage, cur_q, cur_gyro_B, cur_h_moment_B, tar_dgyro_B, tar_thrusts);
}

bool Mixer::setDGyroWeight(double p)
{
  if (p <= 0.)
  {
    cerr << "DGyro weight must be positive." << endl;
    return false;
  }

  dgyro_weight_ = p;
  updateQpWeight();
  return true;
}

bool Mixer::setThrustWeight(double p)
{
  if (p <= 0.)
  {
    cerr << "Thrust weight must be positive." << endl;
    return false;
  }

  thrust_weight_ = p;
  updateQpWeight();
  return true;
}

void Mixer::updateQpWeight()
{
  qp_.problem.P.diagonal().head(3).fill(dgyro_weight_);
  qp_.problem.P.diagonal().tail(z_rotors_.count()).fill(thrust_weight_);
}

void Mixer::updateThrustLimits(const double& cur_voltage, const double& thrusts_sum)
{
  for (size_t i = 0; i < z_rotors_.count(); ++i)
  {
    const auto& rotor = z_rotors_.rotor(i);
    min_thrusts_(i) = rotor.minThrust(cur_voltage);
    max_thrusts_(i) = rotor.maxThrust(cur_voltage);
  }

  // 合計推力の等式制約を満たせない場合は，不等式制約を取り除く
  const auto min_thrusts_sum = min_thrusts_.sum();
  const auto max_thrusts_sum = max_thrusts_.sum();
  if (thrusts_sum < min_thrusts_sum || max_thrusts_sum < thrusts_sum)
  {
    min_thrusts_.fill(0.);
    max_thrusts_.fill(numeric_limits<double>::max());
    if (thrusts_sum < min_thrusts_sum)
      PRINT_DEBUG("Target thrust sum [N] is too small: " << thrusts_sum << " < " << min_thrusts_sum);
    else
      PRINT_DEBUG("Target thrust sum [N] is too large: " << thrusts_sum << " > " << max_thrusts_sum);
  }
}
}  // namespace tobas
