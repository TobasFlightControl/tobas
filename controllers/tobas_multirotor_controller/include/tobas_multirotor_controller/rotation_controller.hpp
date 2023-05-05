#pragma once

#include <Eigen/Core>
#include <kdl/frames.hpp>

#include <dh_linear_control/c2d/rk4.hpp>
#include <dh_linear_control/mpc/linear_dense.hpp>

#include <tobas_tools/rotor_property.hpp>

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
  explicit RotationController(
    const KDL::Tree& tree,
    double gravity,
    double battery_voltage,
    const RotorConfigs& rotor_configs,
    const RotationControllerDynamicParams& params);

  void update(
    const Eigen::Vector3d& cur_rpy,
    const Eigen::Vector3d& cur_angvel,
    const KDL::JntArray& q,
    const double& U,
    const Eigen::Vector3d& tar_rpy,
    Eigen::VectorXd& u_opt);

  void reconfigure(const RotationControllerDynamicParams& params);

private:
  // 定数
  const double gravity_;
  const double battery_voltage_;
  const uint32_t num_rotors_;
  double mass_;

  MultiRotorDynamics cont_;   // 連続時間線形状態方程式
  ctrl::C2D_RK4 c2d_;         // 状態方程式を離散化
  ctrl::LinearDenseMPC mpc_;  // 線形モデル予測制御

  void updateDynamics(
    const Eigen::Vector3d& cur_rpy,
    const Eigen::Vector3d& tar_rpy,
    const KDL::JntArray& q);
  void updateWeight_Q(double rot_weight, double angvel_weight);
  void updateWeight_S(int thrust_weight);
  void updateWeight_R(int thrust_rate_weight, double dt);
  void setInputConstraintBase(const RotorConfigs& rotor_configs);
  void updateInputConstraint(double U);
};
}  // namespace tobas_multirotor_controller
