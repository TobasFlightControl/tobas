#pragma once

#include <tobas_eigen_tools/typedef.hpp>
#include <tobas_quadprog/dual_active_set.hpp>
#include <tobas_kdl/euler.hpp>
#include <tobas_kdl/treefksolverpos.hpp>
#include <tobas_kdl/treejntaxissolver.hpp>
#include <tobas_kdl/treejnttoinertiasolver.hpp>

#include <tobas_drone_core/drone.hpp>
#include <tobas_drone_tools/rotor_axis_extractor.hpp>

namespace tobas
{
struct NonPlanarMixerConfig
{
  double linear_weight;
  double angular_weight;
  double thrust_weight_log10;
};

/**
 * @brief 制約を考慮したマルチコプターの推力ミキシング (memo: 2-43)
 */
class NonPlanarMixer
{
public:
  explicit NonPlanarMixer(const Drone& drone, const kdl::Tree& tree);

  void updateInternalDataStructures();

  Eigen::VectorXd solve(
    const double& cur_voltage,
    const kdl::JntArray& cur_q,
    const kdl::Rotation& cur_rot,
    const kdl::Vector& cur_gyro_B,
    const kdl::Vector& tar_acc_W,
    const kdl::Vector& tar_dgyro_B);

  void configure(const NonPlanarMixerConfig& cfg);

private:
  const Drone& drone_;
  const kdl::Tree& tree_;

  kdl::TreeFkSolverPos fk_solver_;
  kdl::TreeJntAxisSolver jnt_axis_solver_;
  kdl::TreeJntToInertiaSolver inertia_solver_;

  quadprog::DualActiveSetSolver qp_;  // QPソルバー
  Eigen::Diagonal6d Q_;               // EoMの重み
  Eigen::DiagonalXd R_;               // 推力の重み
  Eigen::Matrix6Xd G_;                // EoM行列等式の左辺
  Eigen::Vector6d h_;                 // EoM行列等式の右辺
};
}  // namespace tobas
