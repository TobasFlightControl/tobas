// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_nonplanar_multi_controller/mixer_qp.hpp"

#include <cassert>
#include <ranges>

#include <tobas_constants/scale.hpp>
#include <tobas_eigen_tools/operators.hpp>
#include <tobas_math/core.hpp>
#include <tobas_std_tools/universal_constants.hpp>

namespace tobas::nonplanar_multicopter
{
QpMixer::QpMixer(const Drone& _drone, const kdl::Tree& _tree)
  : super(_drone, _tree), fk_solver_(_tree), inertia_solver_(_tree)
{
}

void QpMixer::updateInternalDataStructures()
{
  super::updateInternalDataStructures();

  fk_solver_.updateInternalDataStructures();
  inertia_solver_.updateInternalDataStructures();

  resizeAndFill();
}

std::expected<Eigen::VectorXd, std::string> QpMixer::solve(
  const kdl::JntArray& _cur_q,
  const kdl::Rotation& _cur_rot,
  const kdl::Vector& _cur_gyro_b,
  const kdl::Vector& _tar_acc_w,
  const kdl::Vector& _tar_dgyro_b,
  const kdl::Vector& _ext_force_w,
  const kdl::Vector& _ext_torque_b)
{
  // Compute forward kinematics.
  const auto& frames = fk_solver_.jntToCart(_cur_q);

  // Compute mass properties.
  const auto inertia = inertia_solver_.jntToCart(_cur_q);
  const auto& mass = inertia.getMass();
  const auto B_Pos_B2G = inertia.getCOG();
  const auto I_B = inertia.getRotationalInertiaCoG();

  // Left-hand side of the EoM matrix equality.
  for (const auto& [idx, rotor_it] : std::views::enumerate(drone_.prop->rotors)) {
    const auto& rotor = rotor_it.second;

    // Compute the rotation axis.
    const auto& elem = tree_.getSegment(rotor->link_name)->second;
    const auto& B_Rot_Par = frames.at(elem.parent->first).M;
    const auto axis_B = B_Rot_Par * elem.segment.joint().axis();

    // Translation.
    G_.block<3, 1>(0, idx) = axis_B.data;

    // Rotation.
    const auto d = rotor->sign();
    const auto cm = rotor->momentConst();
    const auto& B_Pos_B2P = frames.at(rotor->link_name).p;
    const auto B_Pos_G2P = B_Pos_B2P - B_Pos_B2G;
    G_.block<3, 1>(3, idx) = (B_Pos_G2P * axis_B - (d * cm) * axis_B).data;
  }

  // Right-hand side of the translational EoM.
  const kdl::Vector grav_W(0, 0, -st::kGravity);
  auto eom_trans_right_W = mass * (_tar_acc_w - grav_W) - _ext_force_w;  // [N]
  eom_trans_right_W.z(std::max(eom_trans_right_W.z(), 0.0));
  h_.head<3>() = _cur_rot.inverse(eom_trans_right_W).data;

  // Right-hand side of the rotational EoM.
  const auto eom_rot_right_B = I_B * _tar_dgyro_b + _cur_gyro_b * (I_B * _cur_gyro_b) - _ext_torque_b;  // [Nm]
  h_.tail<3>() = eom_rot_right_B.data;

  // Weights for the EoM.
  const auto linear_scale = mass * kAccelScale;                // [N]
  const auto angular_scale = (I_B.trace() / 3) * kDGyroScale;  // [Nm]
  Q_.diagonal().head<3>().fill(cfg_.linear_weight / math::sqr(linear_scale));
  Q_.diagonal().tail<3>().fill(cfg_.angular_weight / math::sqr(angular_scale));

  // Weights for thrust.
  const auto thrust_scale = mass * st::kGravity / drone_.prop->numRotors();  // [N]
  const auto thrust_weight_base = cfg_.thrust_weight / math::sqr(thrust_scale);
  for (const auto& [idx, rotor_it] : std::views::enumerate(drone_.prop->rotors)) {
    const auto& rotor = rotor_it.second;
    R_.diagonal()(idx) = thrust_weight_base * rotor->effortWeight();
  }

  // Objective function.
  qp_.problem.P = G_.transpose() * Q_ * G_ + R_;
  qp_.problem.q = -G_.transpose() * Q_ * h_;

  // Inequality constraints.
  for (const auto& [idx, rotor_it] : std::views::enumerate(drone_.prop->rotors)) {
    const auto& rotor = rotor_it.second;
    if (rotor_alive_[rotor->link_name]) {
      qp_.problem.b(idx) = drone_.prop->maxThrust(rotor->link_name);
      qp_.problem.b(drone_.prop->numRotors() + idx) = -drone_.prop->minThrust(rotor->link_name);
    }
    else {
      qp_.problem.b(idx) = 0.0;
      qp_.problem.b(drone_.prop->numRotors() + idx) = 0.0;
    }
  }

  // Solve the QPP.
  const auto thrusts = qp_.solve();
  if (!thrusts) {
    return std::unexpected("QP failed: " + thrusts.error());
  }

  return thrustDeadband(*thrusts);
}

void QpMixer::setLinearWeight(double _p)
{
  assert(_p > 0.0);
  cfg_.linear_weight = _p;
}

void QpMixer::setAngularWeight(double _p)
{
  assert(_p > 0.0);
  cfg_.angular_weight = _p;
}

void QpMixer::setThrustWeight(double _p)
{
  assert(_p > 0.0);
  cfg_.thrust_weight = _p;
}

void QpMixer::resizeAndFill()
{
  const auto var_size = drone_.prop->numRotors();
  const auto ineq_size = var_size * 2;

  qp_.resize(var_size, 0, ineq_size);
  qp_.setZero();

  qp_.x_scale.setOnes();

  qp_.problem.A.topRows(var_size).diagonal().fill(1);
  qp_.problem.A.bottomRows(var_size).diagonal().fill(-1);

  R_.resize(var_size);
  G_.resize(Eigen::NoChange, var_size);
}
}  // namespace tobas::nonplanar_multicopter
