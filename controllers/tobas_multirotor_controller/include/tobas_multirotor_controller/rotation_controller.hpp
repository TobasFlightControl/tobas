#pragma once

#include <Eigen/Core>
#include <kdl/frames.hpp>

#include <dh_kdl/euler.hpp>
#include <dh_linear_control/c2d/rk4.hpp>
#include <dh_linear_control/mpc/linear_dense.hpp>

#include "./dynamics.hpp"

namespace tobas_multirotor_controller
{
struct RotationControllerDynamicParams
{
  double pred_horizon;
  int pred_steps;
  double rot_decay;
  double angvel_decay;
  double rot_weight;
  double angvel_weight;
  int thrust_weight;
  int thrust_rate_weight;
};

class RotationController
{
public:
  explicit RotationController(const Drone& drone, const RotationControllerDynamicParams& params);

  void update(
    const KDL::Euler& cur_rpy,
    const KDL::Vector& cur_angvel_B,
    const KDL::JntArray& q,
    const double& U,
    const KDL::Euler& tar_rpy,
    Eigen::VectorXd& u_opt);

  void reconfigure(const RotationControllerDynamicParams& params);

private:
  const Drone& drone_;
  std::vector<uint32_t> ver_prop_idxes_;
  uint32_t u_size_;

  MultiRotorDynamics cont_;   // 連続時間線形状態方程式
  ctrl::C2D_RK4 c2d_;         // 状態方程式を離散化
  ctrl::LinearDenseMPC mpc_;  // 線形モデル予測制御

  void updateCurrentState(const KDL::Euler& cur_rpy, const KDL::Vector& cur_angvel_B);
  void updateSetState(const KDL::Euler& tar_rpy);
  void updateDynamics(const KDL::Euler& cur_rpy, const KDL::Euler& tar_rpy, const KDL::JntArray& q);
  void setScales();
  void setInputConstraintBase();
  void updateInputConstraint(double U);
};
}  // namespace tobas_multirotor_controller
