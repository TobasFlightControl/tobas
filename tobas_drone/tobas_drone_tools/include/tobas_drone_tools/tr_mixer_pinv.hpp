#pragma once

#include <tobas_eigen_tools/typedef.hpp>
#include <tobas_kdl/tree_fk_solver_pos_all.hpp>
#include <tobas_kdl/tree_inertia_solver.hpp>

#include <tobas_drone_core/drone.hpp>

namespace tobas
{
/**
 * @brief ティルトロータマルチコプターのミキシングを変数変換で解く． (memo: 3-16)
 */
class TiltRotorMixer_pinv
{
  static constexpr double kSingularThreshLB = 5.;   // [deg]
  static constexpr double kSingularThreshUB = 10.;  // [deg]

public:
  explicit TiltRotorMixer_pinv(const Drone& drone, const kdl::Tree& tree);

  bool updateInternalDataStructures();

  bool solve(
    const kdl::JntArray& cur_q,
    const kdl::Rotation& cur_rot,
    const kdl::Vector& cur_gyro_B,
    const kdl::Vector& tar_acc_W,
    const kdl::Vector& tar_dgyro_B);

  double getThrust(size_t idx) const;
  double getTiltAngle(size_t idx) const;

private:
  const Drone& drone_;
  const kdl::Tree& tree_;

  kdl::TreeFkSolverPosAll fk_solver_;
  kdl::TreeInertiaSolver inertia_solver_;

  std::vector<Eigen::Matrix<double, 3, 2>> A_;
  Eigen::Matrix6Xd E_;
  Eigen::Vector6d f_;
  Eigen::VectorXd x_;

  std::vector<bool> is_singular_;  // 各ティルトロータが特異状態かどうか
};
}  // namespace tobas
