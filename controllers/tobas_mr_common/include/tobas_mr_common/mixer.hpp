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
  double dgyro_weight = 1e+3;
  double thrust_weight = 1.;
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
    const double& cur_voltage,
    const KDL::JntArray& cur_q,
    const Eigen::Vector3d& cur_gyro_B,
    const Eigen::Vector3d& tar_dgyro_B,
    const Eigen::VectorXd& tar_thrusts);

  Eigen::VectorXd solve(
    const double& cur_voltage,
    const KDL::JntArray& cur_q,
    const Eigen::Vector3d& cur_gyro_B,
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

  void updateQpWeight();
};
}  // namespace tobas_mr_common
