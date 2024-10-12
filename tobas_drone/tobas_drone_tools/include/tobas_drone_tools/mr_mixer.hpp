#pragma once

#include <tobas_quadprog/dual_active_set.hpp>
#include <tobas_kdl/treefksolverpos.hpp>
#include <tobas_kdl/treejntaxissolver.hpp>
#include <tobas_kdl/treejnttoinertiasolver.hpp>

#include "./rotor_axis_extractor.hpp"

namespace tobas
{
/**
 * @brief 制約を考慮したマルチコプターの推力ミキシング (memo: 2-43)
 */
class Mixer
{
  static constexpr size_t kEqualityConstSize = 4;

public:
  explicit Mixer(const Drone& drone, const kdl::Tree& tree);

  void updateInternalDataStructures();

  Eigen::VectorXd solve(
    const double& cur_voltage,
    const kdl::JntArray& cur_q,
    const Eigen::Vector3d& cur_gyro_B,
    const Eigen::Vector3d& cur_h_moment_B,
    const Eigen::Vector3d& tar_dgyro_B,
    const Eigen::VectorXd& tar_thrusts);

  Eigen::VectorXd solve(
    const double& cur_voltage,
    const kdl::JntArray& cur_q,
    const Eigen::Vector3d& cur_gyro_B,
    const Eigen::Vector3d& cur_h_moment_B,
    const Eigen::Vector3d& tar_dgyro_B,
    const double& tar_thrusts_sum);

  bool setDGyroWeight(double p);
  bool setThrustWeight(double p);

private:
  const Drone& drone_;
  const kdl::Tree& tree_;

  // QPPの重み
  // 参照推力の実現よりも角加速度の実現を優先すべきか
  double dgyro_weight_ = 1e+3;
  double thrust_weight_ = 1.;

  kdl::TreeFkSolverPos fk_solver_;
  kdl::TreeJntAxisSolver jnt_axis_solver_;
  kdl::TreeJntToInertiaSolver inertia_solver_;
  RotorAxisExtractor z_rotors_;

  quadprog::DualActiveSetSolver qp_;
  Eigen::Matrix3Xd U_;
  Eigen::VectorXd max_thrusts_;
  Eigen::VectorXd min_thrusts_;
  Eigen::VectorXd last_thrusts_;

  void updateQpWeight();
  void updateThrustLimits(const double& cur_voltage, const double& thrusts_sum);
};
}  // namespace tobas
