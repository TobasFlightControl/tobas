// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_legged_tools/swing_leg_controller.hpp"

#include <iostream>

#include <tobas_std_tools/universal_constants.hpp>
#include <tobas_std_tools/vector.hpp>

namespace tobas
{
namespace lr_tools
{
SwingLegController::SwingLegController(
  const kdl::Tree& tree,
  const std::vector<std::string>& thigh_names,
  const std::vector<std::string>& foot_names)
  : tree_(tree)
  , thigh_names_(thigh_names)
  , foot_names_(foot_names)
  , nc_(foot_names.size())
  , fk_solver_(tree)
  , ref_traj_(nc_)
  , is_stand_prev_(nc_)
  , t_switch_(nc_)
  , B_Tdd_BF_(nc_)
  , thigh_0_(nc_)
{
  assert(thigh_names_.size() == foot_names_.size());

  setThighOrigins();
  reset();
}

bool SwingLegController::updateInternalDataStructures()
{
  if (!fk_solver_.updateInternalDataStructures()) {
    return false;
  }

  return true;
}

void SwingLegController::reset()
{
  st::fill(is_stand_prev_, true);
  st::fill(B_Tdd_BF_, kdl::VectorAcc::Zero());  // TODO: Initialize properly.
}

bool SwingLegController::update(
  double z,
  const kdl::Vector& G_Vel_GB,
  const kdl::Rotation& W_Rot_B,
  const kdl::Vector& G_Gyro_GB,
  const kdl::JntArray& q,
  const std::vector<bool>& is_stand,
  const TimeType& cur_time)
{
  if (is_stand.size() != nc_) {
    std::cerr << "The size of is_stand mismatch." << std::endl;
    return false;
  }

  W_Rot_B.getRPY(roll_, pitch_, yaw_);
  const auto G_Rot_B = kdl::Rotation::RPY(roll_, pitch_, 0.);

  for (size_t l = 0; l < nc_; ++l) {
    /* ===== Generate a trajectory when transitioning from stance to swing ===== */
    if (is_stand_prev_[l] && !is_stand[l]) {
      t_switch_[l] = cur_time;

      // Compute the initial position of {foot} relative to {gnd}, viewed from {gnd}.
      if (fk_solver_.jntToCart(q, foot_names_[l]) < 0) {
        std::cerr << "FK failed: " << fk_solver_.errorMessage() << std::endl;
        return false;
      }
      const auto& B_Pos_BF = fk_solver_.getFrame().p;
      const kdl::Vector G_Pos_GB(0, 0, z);
      const auto G_Pos_GF_0 = G_Pos_GB + G_Rot_B * B_Pos_BF;

      // Compute the final position of {foot} relative to {gnd}, viewed from {gnd}.
      // Capture Gain
      // cf. MIT Cheetah 3: Design and Control of a Robust, Dynamic Quadruped Robot
      // https://ieeexplore.ieee.org/abstract/document/8593885
      const auto capture_gain = sqrt(std::max(z, 0.) / st::kGravity);

      // (12) ~ (15): Only xy needs to match.
      const kdl::Vector tar_vel(vx_, vy_, 0.);
      const kdl::Vector tar_gyro(0., 0., yawrate_);
      const auto p_thigh = G_Rot_B * thigh_0_[l];  // `G_Pos_B2Thigh0`; only xy needs to match.
      const auto p_sym = (stand_period_ / 2) * tar_vel + raibert_gain_ * (G_Vel_GB - tar_vel);
      const auto p_cent = (capture_gain / 2) * (p_thigh * tar_gyro);
      const auto G_Pos_GF_f = p_thigh + p_sym + p_cent;

      // Trajectory generation.
      if (!ref_traj_[l].generate(G_Pos_GF_0, G_Pos_GF_f, swing_period_, clearance_)) {
        return false;
      }
    }

    /* ===== Update contact states ===== */
    is_stand_prev_[l] = is_stand[l];

    /* ===== Update the target swing-leg foot-tip state ===== */
    // Keep the previous value during stance.
    if (is_stand[l]) {
      continue;
    }

    // Get the target state of each foot tip viewed from {gnd}.
    const auto t = std::max(DurationType(cur_time - t_switch_[l]).count(), 0.);
    if (!ref_traj_[l].get(t, G_Tdd_GF_.p, G_Tdd_GF_.v, G_Tdd_GF_.dv)) {
      return false;
    }

    // Compute the state of {foot} relative to {bs}, viewed from {gnd}.
    const kdl::Vector G_Pos_GB(0, 0, z);
    const auto G_Tdd_BF = G_Tdd_GF_ - G_Pos_GB;

    // Convert to the target state of each foot tip viewed from {bs} (memo: 1-50).
    B_Tdd_BF_[l].p = G_Rot_B.inverse(G_Tdd_BF.p);
    B_Tdd_BF_[l].v = G_Rot_B.inverse(G_Tdd_BF.v - G_Gyro_GB * G_Tdd_BF.p);
    B_Tdd_BF_[l].dv = G_Rot_B.inverse(G_Tdd_BF.dv - 2 * G_Gyro_GB * G_Tdd_BF.v);
  }

  return true;
}

bool SwingLegController::setRaibertGain(double raibert_gain)
{
  if (raibert_gain <= 0.) {
    std::cerr << "Raibert gain must be positive." << std::endl;
    return false;
  }

  raibert_gain_ = raibert_gain;
  return true;
}

bool SwingLegController::setClearance(double clearance)
{
  if (clearance <= 0.) {
    std::cerr << "Foot clearance must be positive." << std::endl;
    return false;
  }

  clearance_ = clearance;
  return true;
}

bool SwingLegController::setGaitParams(double stand_period, double swing_period)
{
  if (stand_period <= 0.) {
    std::cerr << "Stand period must be positive." << std::endl;
    return false;
  }
  if (swing_period <= 0.) {
    std::cerr << "Swing period must be positive." << std::endl;
    return false;
  }

  stand_period_ = stand_period;
  swing_period_ = swing_period;
  return true;
}

bool SwingLegController::setVelocity(double vx, double vy, double yawrate)
{
  vx_ = vx;
  vy_ = vy;
  yawrate_ = yawrate;
  return true;
}

void SwingLegController::setThighOrigins()
{
  const auto q0 = kdl::JntArray::Zero(tree_.getNrOfJoints());

  for (size_t l = 0; l < nc_; ++l) {
    if (fk_solver_.jntToCart(q0, thigh_names_[l]) < 0) {
      throw std::runtime_error("FK failed: " + fk_solver_.errorMessage());
    }
    thigh_0_[l] = fk_solver_.getFrame().p;
  }
}
}  // namespace lr_tools
}  // namespace tobas
