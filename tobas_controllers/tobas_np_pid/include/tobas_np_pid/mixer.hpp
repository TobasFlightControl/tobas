#pragma once

#include <tobas_eigen_tools/typedef.hpp>
#include <tobas_quadprog/dual_active_set.hpp>
#include <tobas_kdl/euler.hpp>
#include <tobas_kdl/treefksolverpos.hpp>
#include <tobas_kdl/treejntaxissolver.hpp>
#include <tobas_kdl/treejnttoinertiasolver.hpp>

#include <tobas_tools/drone.hpp>
#include <tobas_tools/rotor_axis_extractor.hpp>

namespace tobas_np_pid
{
struct MixerConfig
{
  double linear_weight;
  double angular_weight;
  double thrust_weight_log10;
};

/**
 * @brief 制約を考慮したマルチコプターの推力ミキシング (memo: 2-43)
 */
class Mixer
{
public:
  explicit Mixer(const tobas::Drone& drone);

  void updateInternalDataStructures();

  Eigen::VectorXd solve(
    const double& cur_voltage,
    const tobas_kdl::JntArray& cur_q,
    const tobas_kdl::Rotation& cur_rot,
    const tobas_kdl::Vector& cur_gyro_B,
    const tobas_kdl::Vector& tar_acc_W,
    const tobas_kdl::Vector& tar_dgyro_B);

  void configure(const MixerConfig& cfg);

private:
  const tobas::Drone& drone_;

  tobas_kdl::TreeFkSolverPos fk_solver_;
  tobas_kdl::TreeJntAxisSolver jnt_axis_solver_;
  tobas_kdl::TreeJntToInertiaSolver inertia_solver_;

  quadprog::DualActiveSetSolver qp_;  // QPソルバー
  Eigen::Diagonal6d Q_;               // EoMの重み
  Eigen::DiagonalXd R_;               // 推力の重み
  Eigen::Matrix6Xd G_;                // EoM行列等式の左辺
  Eigen::Vector6d h_;                 // EoM行列等式の右辺
};
}  // namespace tobas_np_pid
