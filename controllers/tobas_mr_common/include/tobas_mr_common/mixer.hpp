#pragma once

#include <dh_quadprog/dual_active_set.hpp>
#include <dh_kdl/treefksolverpos.hpp>
#include <dh_kdl/treejnttoinertiasolver.hpp>

#include <tobas_tools/drone.hpp>
#include <tobas_tools/rotor_axis_extractor.hpp>

namespace tobas_mr_common
{
struct MixerConfig
{
  // QPPの重み．参照推力の実現よりも角加速度の実現を優先すべきか．
  double dgyro_weight = 1e+3;
  double thrust_weight = 1.;

  // 回転数の変化率の最大値 [rad/s^2]
  // 回転数の変化率に制限をかけると推力和の等式条件を満たす解が存在しなくなる恐れがある
  // また，クランプ自体が望ましい挙動ではないため，ハード制約ではなくDゲインやソフト制約で調整する方が有効
  double max_rot_acc = std::numeric_limits<double>::max();
};

/**
 * @brief 制約を考慮したマルチコプターの推力ミキシング (memo: 2-43)
 */
class Mixer
{
  static constexpr uint32_t kEqualityConstSize = 4;

public:
  explicit Mixer(const tobas::Drone& drone);

  void updateInternalDataStructures();

  Eigen::VectorXd solve(
    const double& dt,
    const double& cur_voltage,
    const KDL::JntArray& cur_q,
    const Eigen::Vector3d& cur_gyro_B,
    const Eigen::Vector3d& cur_h_moment_B,
    const Eigen::Vector3d& tar_dgyro_B,
    const Eigen::VectorXd& tar_thrusts);

  Eigen::VectorXd solve(
    const double& dt,
    const double& cur_voltage,
    const KDL::JntArray& cur_q,
    const Eigen::Vector3d& cur_gyro_B,
    const Eigen::Vector3d& cur_h_moment_B,
    const Eigen::Vector3d& tar_dgyro_B,
    const double& tar_thrusts_sum);

  void configure(const MixerConfig& cfg);

private:
  const tobas::Drone& drone_;

  KDL::TreeFkSolverPos fk_solver_;
  KDL::TreeJntToInertiaSolver inertia_solver_;
  tobas::RotorAxisExtractor z_rotors_;

  MixerConfig cfg_;

  quadprog::DualActiveSetSolver qp_solver_;

  Eigen::Matrix3Xd A_;
  Eigen::VectorXd max_thrusts_;
  Eigen::VectorXd min_thrusts_;
  Eigen::VectorXd last_thrusts_;

  void updateQpWeight();
  void updateThrustLimits(const double& dt, const double& cur_voltage, const double& thrusts_sum);
};
}  // namespace tobas_mr_common
