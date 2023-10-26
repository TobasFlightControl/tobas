#pragma once

#include <Eigen/Core>

#include <dh_std_tools/stopwatch.hpp>
#include <dh_kdl/frames.hpp>
#include <dh_kdl/euler.hpp>
#include <dh_linear_control/c2d/tustin.hpp>
#include <dh_linear_control/mpc/linear_dense.hpp>

#include <tobas_mr_common/dynamics.hpp>
#include <tobas_mr_common/mixer.hpp>

#include "./dynamics.hpp"

namespace tobas_mr_mpc
{
/**
 * @brief RotationControllerの動的パラメータをまとめた構造体．
 *
 * @note 姿勢角をattitude (roll + pitch) とheading (yaw) を分けているのは，
 * 一般に前者の方が後者に比べて重要度が高いため．
 */
struct OrientationControllerConfig
{
  double max_attitude;       // [rad]
  double max_heading_error;  // [rad]
  double h_force_comp_rate;  // H-forceの理論値に対する補償項の割合 [0, 1]
  double kp;                 // モデル化誤差を含む外乱補償用のPゲイン
  double kd;                 // モデル化誤差を含む外乱補償用のDゲイン

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

class OrientationController
{
public:
  explicit OrientationController(const tobas::Drone& drone);

  void updateInternalDataStructures();

  Eigen::VectorXd solve(
    const KDL::Euler& cur_rpy,
    const KDL::Twist& cur_twist_B,
    const KDL::Vector& cur_wind_W,
    const KDL::JntArray& cur_q,
    const double& cur_voltage,
    const std::vector<double>& cur_rot_speeds,
    const double& tar_U,
    KDL::Euler tar_rpy);

  void configure(const OrientationControllerConfig& config);

  const Eigen::VectorXd& mpcThrusts() const;

private:
  const tobas::Drone& drone_;

  tobas::RotorAxisExtractor z_rotors_;
  tobas_mr_common::MultirotorDynamicsComponents dynamics_;
  tobas_mr_common::Mixer mixer_;

  MultiRotorDynamics cont_;
  ctrl::C2D_Tustin c2d_;
  ctrl::LinearDenseMPC mpc_;

  // Config
  double max_attitude_;
  double max_heading_error_;
  double h_force_comp_rate_;
  double kp_;
  double kd_;

  dh_std::Stopwatch stopwatch_;


  void updateCurrentState(
    const KDL::Euler& cur_rpy,
    const KDL::Twist& cur_twist_B,
    const KDL::Vector& cur_wind_W,
    const KDL::JntArray& cur_q,
    const double& thrust_z);
  void updateSetState(const KDL::Euler& tar_rpy);
  void fillInputConstraintFixedParts();
};
}  // namespace tobas_mr_mpc
