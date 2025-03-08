#pragma once

#include <tobas_eigen_tools/typedef.hpp>
#include <tobas_kdl/tree_fk_solver_pos_all.hpp>
#include <tobas_kdl/tree_inertia_solver.hpp>

#include "./mixer.hpp"

namespace tobas
{
/**
 * @brief ティルトロータマルチコプターのミキシングを変数変換で解く． (memo: 3-16)
 */
class TiltRotorMixer_pinv : public Mixer
{
  using super = Mixer;

  static constexpr double kMinVerticalForcePerMass = 5.;  // [m/s^2]

public:
  explicit TiltRotorMixer_pinv(const Drone& drone, const kdl::Tree& tree);

  bool updateInternalDataStructures() override;

  bool solve(
    const kdl::JntArray& cur_q,
    const kdl::Rotation& cur_rot,
    const kdl::Vector& cur_gyro_B,
    const kdl::Vector& tar_acc_W,
    const kdl::Vector& tar_dgyro_B,
    const kdl::Vector& ext_force_W = kdl::Vector::Zero(),
    const kdl::Vector& ext_torque_B = kdl::Vector::Zero());

  double getThrust(size_t idx) const;
  double getTiltAngle(size_t idx) const;

  bool setTiltAxisSingularDeclinationLB(double lb_rad);
  bool setTiltAxisSingularDeclinationUB(double ub_rad);

private:
  struct Config
  {
    double singular_declination_lb = 0.;  // [rad]
    double singular_declination_ub = 0.;  // [rad]
  } cfg_;

  kdl::TreeFkSolverPosAll fk_solver_;
  kdl::TreeInertiaSolver inertia_solver_;

  std::vector<Eigen::Matrix<double, 3, 2>> A_;
  Eigen::Matrix6Xd E_;
  Eigen::Vector6d f_;
  Eigen::VectorXd x_;

  std::map<std::string, bool> is_singular_;  // 各ティルトロータが特異状態かどうか
};
}  // namespace tobas
