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
    const KDL::JntArray& cur_q,
    const KDL::Rotation& cur_rot,
    const KDL::Vector& cur_gyro_B,
    const KDL::Vector& tar_acc_W,
    const KDL::Vector& tar_dgyro_B);

  void configure(const MixerConfig& cfg);

private:
  const tobas::Drone& drone_;

  KDL::TreeFkSolverPos fk_solver_;
  KDL::TreeJntAxisSolver jnt_axis_solver_;
  KDL::TreeJntToInertiaSolver inertia_solver_;

  quadprog::DualActiveSetSolver qp_;     // QPソルバー
  Eigen::Diagonal6d Q_;                  // EoMの重み
  Eigen::DiagonalXd R_;                  // 推力の重み
  Eigen::Matrix6Xd G_;                   // EoM行列等式の左辺
  Eigen::Vector6d h_;                    // EoM行列等式の右辺
  std::vector<KDL::Vector> cog2prop_B_;  // Translation from CoG to propellers wrt. base frame
  std::vector<KDL::Vector> axis_B_;      // Rotating axes wrt. base frame
};
}  // namespace tobas_np_pid
