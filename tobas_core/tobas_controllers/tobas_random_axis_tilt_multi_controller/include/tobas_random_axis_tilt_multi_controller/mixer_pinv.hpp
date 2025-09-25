#pragma once

#include <tobas_drone_tools/mixer_i.hpp>
#include <tobas_eigen_tools/typedef.hpp>
#include <tobas_kdl/tree_fk_solver_pos_all.hpp>
#include <tobas_kdl/tree_inertia_solver.hpp>

namespace tobas
{
namespace random_axis_tilt_multicopter
{
/* チルトロータマルチコプターのミキシングを変数変換で解く (memo: 3-16)． */
class PinvMixer : public MixerI
{
  using super = MixerI;

public:
  explicit PinvMixer(const Drone& drone, const kdl::Tree& tree);

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

  std::map<std::string, kdl::Vector> thrust_points_;  // 祖父母リンクから見た推力の作用点
  std::map<std::string, bool> is_singular_;           // 各チルトロータが特異状態かどうか
};
}  // namespace random_axis_tilt_multicopter
}  // namespace tobas
