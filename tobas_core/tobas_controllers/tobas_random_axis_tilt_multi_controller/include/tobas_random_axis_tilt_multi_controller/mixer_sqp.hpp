// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_eigen_tools/tensor.hpp>
#include <tobas_eigen_tools/typedef.hpp>
#include <tobas_kdl/tree_fk_solver_pos_all.hpp>
#include <tobas_kdl/tree_inertia_solver.hpp>
#include <tobas_kdl/tree_joint_parser.hpp>
#include <tobas_nlp/sqp.hpp>
#include <tobas_nonplanar_multi_controller/mixer_qp.hpp>
#include <tobas_tools/mixer_i.hpp>

#include "./mixer_solution.hpp"

namespace tobas::random_axis_tilt_multicopter
{
/** Solve tilt-rotor multicopter mixing with SQP (memo: 3-12). */
class SqpMixer : public MixerI
{
  using self = SqpMixer;
  using super = MixerI;

public:
  explicit SqpMixer(const Drone& _drone, const kdl::Tree& _tree);

  void updateInternalDataStructures() override;

  std::expected<MixerSolution, std::string> solve(
    const kdl::JntArray& _cur_q,
    const kdl::Rotation& _cur_rot,
    const kdl::Vector& _cur_gyro_b,
    const kdl::Vector& _tar_acc_w,
    const kdl::Vector& _tar_dgyro_b,
    const kdl::Vector& _ext_force_w = kdl::Vector::Zero(),
    const kdl::Vector& _ext_torque_b = kdl::Vector::Zero());

  void setLinearWeight(double _p);
  void setAngularWeight(double _p);
  void setThrustWeight(double _p);

private:
  struct Config
  {
    double linear_weight = 1.0;
    double angular_weight = 1.0;
    double thrust_weight = 1e-6;
  } cfg_;

  kdl::TreeJointParser joint_parser_;
  kdl::TreeFkSolverPosAll fk_solver_;
  kdl::TreeInertiaSolver inertia_solver_;
  nonplanar_multicopter::QpMixer np_mixer_;

  nlp::SQP sqp_;
  const kdl::FrameMap* frames_ = nullptr;  ///< Owned by `fk_solver_`; reused by the SQP callbacks.

  Eigen::Diagonal6d Q_;  ///< EoM weights.
  Eigen::DiagonalXd R_;  ///< Thrust weights.
  Eigen::Matrix6Xd B_;
  Eigen::Vector6d d_;
  Eigen::MatrixXd N_;
  Eigen::Tensor3Xd dN_dtheta_;
  Eigen::Tensor4Xd dN_dtheta_2_;
  Eigen::MatrixXd Ci_;
  Eigen::VectorXd ci0_;
  Eigen::Tensor3Xd dCi_dx_;
  Eigen::RowVectorXd df_dx_;
  Eigen::MatrixXd df_dx_2_;

  void resetTensors();
  void initializeSQP();

  // Functions passed to SQP.
  double f(const Eigen::VectorXd& _x);
  Eigen::VectorXd g(const Eigen::VectorXd& _x);
  Eigen::VectorXd h(const Eigen::VectorXd& _x);
  Eigen::RowVectorXd dfdx(const Eigen::VectorXd& _x);
  Eigen::MatrixXd dgdx(const Eigen::VectorXd& _x);
  Eigen::MatrixXd dhdx(const Eigen::VectorXd& _x);
  Eigen::MatrixXd dFdx(const Eigen::VectorXd&);
  Eigen::Tensor3Xd dGdx(const Eigen::VectorXd&);
  Eigen::Tensor3Xd dHdx(const Eigen::VectorXd& _x);

  size_t stateSize() const;
  std::pair<Eigen::VectorXd, Eigen::VectorXd> splitState(const Eigen::VectorXd& _x) const;

  Eigen::Vector6d calc_e(const Eigen::VectorXd& _theta, const Eigen::VectorXd& _tau);
  Eigen::Vector6d calc_u(const Eigen::VectorXd& _theta, const Eigen::VectorXd& _tau);
  Eigen::Matrix6Xd calc_C(const Eigen::VectorXd& _theta);
  const Eigen::MatrixXd& calc_N(const Eigen::VectorXd& _theta);
  const Eigen::Tensor3Xd& calc_dN_dtheta(const Eigen::VectorXd& _theta);
  const Eigen::Tensor4Xd& calc_dN_dtheta_2(const Eigen::VectorXd& _theta);
  Eigen::Matrix6Xd calc_du_dtheta(const Eigen::VectorXd& _theta, const Eigen::VectorXd& _tau);
  Eigen::Tensor3Xd calc_du_dtheta_2(const Eigen::VectorXd& _theta, const Eigen::VectorXd& _tau);
  Eigen::Tensor3Xd calc_dC_dtheta(const Eigen::VectorXd& _theta);
  Eigen::Tensor4Xd calc_dC_dtheta_2(const Eigen::VectorXd& _theta);
};
}  // namespace tobas::random_axis_tilt_multicopter
