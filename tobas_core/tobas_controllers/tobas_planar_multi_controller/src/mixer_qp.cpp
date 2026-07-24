// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_planar_multi_controller/mixer_qp.hpp"

#include <ranges>

#include <tobas_constants/scale.hpp>
#include <tobas_eigen_tools/operators.hpp>
#include <tobas_math/core.hpp>
#include <tobas_std_tools/universal_constants.hpp>

namespace tobas
{
namespace planar_multicopter
{
QpMixer::QpMixer(const Drone& drone, const kdl::Tree& tree)
  : super(drone, tree), fk_solver_(tree), inertia_solver_(tree), stopwatch_(100)
{
}

bool QpMixer::updateInternalDataStructures()
{
  if (!super::updateInternalDataStructures()) {
    return false;
  }

  if (!fk_solver_.updateInternalDataStructures()) {
    return false;
  }
  if (!inertia_solver_.updateInternalDataStructures()) {
    return false;
  }

  resizeAndFill();

  return true;
}

bool QpMixer::solve(
  const kdl::JntArray& cur_q,
  const kdl::Vector& cur_gyro_B,
  const kdl::Vector& tar_dgyro_B,
  const double& tar_thrusts_sum,
  const kdl::Vector& ext_torque_B)
{
  if (tar_thrusts_sum < 0.0) {
    std::cerr << "Target thrust must be non-negative: " << tar_thrusts_sum << " < 0" << std::endl;
    return false;
  }

  // Compute forward kinematics.
  if (fk_solver_.jntToCart(cur_q) < 0) {
    std::cerr << "Forward kinematics failed: " << fk_solver_.errorMessage() << std::endl;
    return false;
  }

  // Compute mass properties.
  if (inertia_solver_.jntToCart(cur_q) < 0) {
    std::cerr << "Inertia solver failed: " << inertia_solver_.errorMessage() << std::endl;
    return false;
  }
  const auto& inertia = inertia_solver_.getInertia();
  const auto& mass = inertia.getMass();
  const auto B_Pos_B2G = inertia.getCOG();
  const auto I_B = inertia.getRotationalInertiaCoG();

  // Left-hand side of the EoM matrix equality.
  for (const auto& [idx, pair] : std::views::enumerate(drone_.prop->rotors)) {
    const auto& rotor = pair.second;

    const auto& B_Pos_B2P = fk_solver_.getFrame(rotor->link_name).p;

    const auto& elem = tree_.getSegment(rotor->link_name)->second;
    const auto& B_Rot_Par = fk_solver_.getFrame(elem.parent->first).M;
    const auto axis_B = B_Rot_Par * elem.segment.joint().axis();

    const auto d = rotor->sign();
    const auto cm = rotor->momentConst();
    const auto B_Pos_G2P = B_Pos_B2P - B_Pos_B2G;
    G_.col(idx) = (B_Pos_G2P * axis_B - (d * cm) * axis_B).data;
  }

  // Right-hand side of the EoM matrix equality.
  h_ = (I_B * tar_dgyro_B + cur_gyro_B * (I_B * cur_gyro_B) - ext_torque_B).data;  // [Nm]

  // Weights for the EoM.
  const auto angular_scale = (I_B.trace() / 3) * kDGyroScale;  // [Nm]
  Q_.diagonal().fill(cfg_.base_weight / math::sqr(angular_scale));

  // Weights for thrust.
  const auto thrust_scale = mass * st::kGravity / drone_.prop->numRotors();  // [N]
  const auto thrust_weight_base = cfg_.thrust_weight / math::sqr(thrust_scale);
  for (const auto& [idx, pair] : std::views::enumerate(drone_.prop->rotors)) {
    const auto& rotor = pair.second;
    R_.diagonal()(idx) = thrust_weight_base * rotor->effortWeight();
  }

  // Objective function.
  qp_.problem.P = G_.transpose() * Q_ * G_ + R_;
  qp_.problem.q = -G_.transpose() * Q_ * h_;

  // Inequality constraints.
  // Compute the total thrust range at the same time.
  double max_thrust_sum = 0.0;
  double min_thrust_sum = 0.0;
  for (const auto& [idx, pair] : std::views::enumerate(drone_.prop->rotors)) {
    const auto& rotor = pair.second;

    double max_thrust, min_thrust;
    if (rotor_alive_[rotor->link_name]) {
      max_thrust = drone_.prop->maxThrust(rotor->link_name);
      min_thrust = drone_.prop->minThrust(rotor->link_name);
    }
    else {
      max_thrust = 0.0;
      min_thrust = 0.0;
    }

    qp_.problem.b(idx) = max_thrust;
    qp_.problem.b(drone_.prop->numRotors() + idx) = -min_thrust;
    max_thrust_sum += max_thrust;
    min_thrust_sum += min_thrust;
  }

  // Exit if thrust cannot be generated.
  if (max_thrust_sum == 0.0) {
    std::cerr << "The vehicle cannot generate thrust." << std::endl;
    return false;
  }

  // Equality constraints.
  // Clamp so that it does not conflict with inequality constraints.
  constexpr double kThrustMargin = 1e-3;  // [N]
  qp_.problem.h(0) = std::clamp(tar_thrusts_sum, min_thrust_sum + kThrustMargin, max_thrust_sum - kThrustMargin);

  // Solve the QPP.
  // stopwatch_.start();
  if (!qp_.solve()) {
    std::cerr << "QP failed: " << qp_.errorMessage() << std::endl;
    return false;
  }
  // stopwatch_.stop();

  return true;
}

double QpMixer::getThrust(size_t idx) const
{
  return thrustDeadband(qp_.solution()(idx));
}

bool QpMixer::setBaseWeight(double p)
{
  if (p <= 0.0) {
    std::cerr << "Base weight must be positive." << std::endl;
    return false;
  }

  cfg_.base_weight = p;
  return true;
}

bool QpMixer::setThrustWeight(double p)
{
  if (p <= 0.0) {
    std::cerr << "Thrust weight must be positive." << std::endl;
    return false;
  }

  cfg_.thrust_weight = p;
  return true;
}

void QpMixer::resizeAndFill()
{
  const auto var_size = drone_.prop->numRotors();

  qp_.resize(var_size, 1, var_size * 2);
  qp_.setZero();

  qp_.x_scale.setOnes();

  qp_.problem.G.fill(1);
  qp_.problem.A.topRows(var_size).diagonal().fill(1);
  qp_.problem.A.bottomRows(var_size).diagonal().fill(-1);

  R_.resize(var_size);
  G_.resize(Eigen::NoChange, var_size);
}
}  // namespace planar_multicopter
}  // namespace tobas
