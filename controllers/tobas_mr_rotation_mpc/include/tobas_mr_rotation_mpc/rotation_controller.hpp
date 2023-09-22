#pragma once

#include <Eigen/Core>
#include <kdl/frames.hpp>

#include <dh_kdl/euler.hpp>
#include <dh_kdl/treefksolverpos.hpp>
#include <dh_kdl/treejnttoinertiasolver.hpp>
#include <dh_linear_control/c2d/tustin.hpp>
#include <dh_linear_control/c2d/rk4.hpp>
#include <dh_linear_control/mpc/linear_dense.hpp>

#include "./dynamics.hpp"

namespace tobas_mr_rotation_mpc
{
/**
 * @brief RotationControllerの動的パラメータをまとめた構造体．
 *
 * @note 姿勢角をattitude (roll + pitch) とheading (yaw) を分けているのは，
 * 一般に前者の方が後者に比べて重要度が高いため．
 */
struct RotationControllerConfig
{
  double max_attitude;       // [rad]
  double max_heading_error;  // [rad]
  double h_force_comp_rate;  // H-forceの理論値に対する補償項の割合 [0, 1]

  double pred_horizon;
  int pred_steps;
  double attitude_decay;
  double heading_decay;
  double angvel_decay;
  double attitude_weight;
  double heading_weight;
  double angvel_weight;
  int thrust_rate_weight_log10;
};

class RotationController
{
public:
  explicit RotationController(const tobas::Drone& drone);

  void updateInternalDataStructures();

  void update(
    const KDL::Euler& cur_rpy,
    const KDL::Twist& cur_twist_B,
    const KDL::JntArray& q,
    double battery_voltage,
    double tar_U,
    const KDL::Euler& tar_rpy,
    Eigen::VectorXd& u_opt);

  void configure(const RotationControllerConfig& params);

private:
  const tobas::Drone& drone_;

  KDL::ExtTreeFkSolverPos fk_solver_;
  KDL::TreeJntToInertiaSolver inertia_solver_;
  tobas::RotorAxisExtractor z_rotors_;

  MultiRotorDynamics cont_;
  ctrl::C2D_Tustin c2d_;
  // ctrl::C2D_RK4 c2d_;
  ctrl::LinearDenseMPC mpc_;

  KDL::Frame T_base_rotor_;
  KDL::Vector P_base_cog_;
  KDL::RotationalInertia I_cog_;  // CoG周りの回転慣性テンソル

  double max_attitude_;
  double max_heading_error_;
  double h_force_coef_;

  double maxThrustSum(double battery_voltage) const;
  double minThrustSum(double battery_voltage) const;
  void updateCurrentState(
    const KDL::Euler& cur_rpy,
    const KDL::Twist& cur_twist_B,
    const KDL::JntArray& q,
    double thrust_z);
  void updateSetState(double tar_roll, double tar_pitch, double tar_yaw);
  void fillInputConstraintFixedParts();
};
}  // namespace tobas_mr_rotation_mpc
