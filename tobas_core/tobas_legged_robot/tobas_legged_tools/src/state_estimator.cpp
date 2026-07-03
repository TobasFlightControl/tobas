// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_legged_tools/state_estimator.hpp"

#include <tobas_std_tools/universal_constants.hpp>

#define EPS 1e-9

namespace tobas
{
namespace lr_tools
{
StateEstimator::StateEstimator(const kdl::Tree& tree, const std::vector<std::string>& foot_names)
  : foot_names_(foot_names)
  , nc_(foot_names.size())
  , fk_solver_(tree)
  , cont_(tree, foot_names)
  , kf_(cont_.stateSize(), cont_.inputSize(), 6 + 4 * nc_, cont_.stateSize())
  , c2d_(cont_.stateSize(), cont_.inputSize())
{
  initializeKalmanFilter();
}

bool StateEstimator::updateInternalDataStructures()
{
  if (!fk_solver_.updateInternalDataStructures()) {
    return false;
  }
  if (!cont_.updateInternalDataStructures()) {
    return false;
  }

  initializeKalmanFilter();
  return true;
}

bool StateEstimator::configure(const StateEstimatorConfig& cfg)
{
  if (cfg.variance_coef <= 0) {
    return false;
  }

  cfg_ = cfg;
  return true;
}

void StateEstimator::update(
  const kdl::Quaternion& W_Quat_B,
  const kdl::Vector& gyro_B,
  const kdl::JntArray& q,
  const kdl::JntArray& qd,
  const std::vector<bool>& is_stand,
  const std::vector<double>& contact_probs,
  const std::vector<kdl::Vector>& foot_forces,
  const std::vector<double>& foot_torques,
  const double& dt)
{
  assert(is_stand.size() == nc_);
  assert(contact_probs.size() == nc_);
  assert(foot_forces.size() == nc_);
  assert(foot_forces.size() == nc_);
  assert(dt >= 0);

  /* ===== Precomputation ===== */
  W_Quat_B.getRPY(roll_, pitch_, yaw_);
  const auto FP_Rot_B = kdl::Rotation::RPY(roll_, pitch_, 0);
  const auto gyro_FP = FP_Rot_B * gyro_B;

  /* ===== Update dynamics ===== */
  cont_.update(roll_, pitch_, q, is_stand);
  const auto disc_dyn = c2d_.convert(cont_, dt);
  kf_.ss.updateDynamics(disc_dyn);

  /* ===== Update the observation-noise covariance matrix ===== */
  // Euler angles.
  kf_.R(kRollIdx, kRollIdx) = EPS;
  kf_.R(kPitchIdx, kPitchIdx) = EPS;

  // Rotational velocity.
  kf_.R.diagonal().segment<3>(kGyroIdx).fill(EPS);

  // Gravity.
  kf_.R(kGravIdx, kGravIdx) = EPS;

  for (size_t l = 0; l < nc_; ++l) {
    // Compute variance.
    double var;
    if (is_stand[l]) {
      var = std::max(cfg_.variance_coef * std::pow(1 - contact_probs[l], cfg_.variance_exp), EPS);
    }
    else {
      var = std::numeric_limits<double>::max();
    }

    // Height from the ground.
    kf_.R(altIdx(l), altIdx(l)) = var;

    // Translational velocity.
    kf_.R.diagonal().segment<3>(velIdx(l)).fill(var);
  }

  /* ===== Update the output vector ===== */
  // Euler angles.
  kf_.y(kRollIdx) = roll_;
  kf_.y(kPitchIdx) = pitch_;

  // Rotational velocity.
  kf_.y.segment<3>(kGyroIdx) = gyro_FP.data;

  // Gravity.
  kf_.y(kGravIdx) = st::kGravity;

  for (size_t l = 0; l < nc_; ++l) {
    if (is_stand[l]) {
      // Forward kinematics.
      if (fk_solver_.jntToCart(q, qd, foot_names_[l]) < 0) {
        throw std::runtime_error("FK failed: " + fk_solver_.errorMessage());
      }
      const auto& foot_pos = fk_solver_.getFrameVel().p.p;
      const auto& foot_vel = fk_solver_.getFrameVel().p.v;

      // For stance legs, estimate the height from the ground by negating the z coordinate of the foot tip.
      // A simple offset is added because there is a steady-state error.
      const auto foot_height = (FP_Rot_B * foot_pos).z();
      kf_.y(altIdx(l)) = -foot_height + kFootGroundOffset;

      // Translational velocity (memo: 1-28).
      kf_.y.segment<3>(velIdx(l)) = -(FP_Rot_B * (gyro_B * foot_pos + foot_vel)).data;
    }
    else {
      // For swing legs, use the predicted state directly as the observed state.
      // TODO: Consider a better estimation method.
      kf_.y(altIdx(l)) = kf_.state()(LinearDynamics::kAltIdx);
      kf_.y.segment<3>(velIdx(l)) = kf_.state().segment<3>(LinearDynamics::kVelXIdx);
    }
  }

  /* ===== Update control input ===== */
  // FIXME: If state estimation depends on the control input, it may diverge.
  for (size_t l = 0; l < nc_; ++l) {
    kf_.u.segment<3>(cont_.forceIndex(l)) = foot_forces[l].data;
    kf_.u(cont_.torqueIndex(l)) = foot_torques[l];
  }

  /* ===== Advance the Kalman filter by one step ===== */
  kf_.update();
}

void StateEstimator::initializeKalmanFilter()
{
  kf_.setZero();

  kf_.ss.C = makeCy();
  kf_.Bv.diagonal().setOnes();  // System noise is assumed to be added directly.
  kf_.Q.diagonal().fill(EPS);

  Eigen::VectorXd init_x(cont_.stateSize());
  init_x << 0, 0, kInitTrunkHeight, 0, 0, 0, 0, 0, 0, st::kGravity;  // FIXME: Estimate the initial trunk height.
  kf_.initialize(init_x, Eigen::MatrixXd::Identity(cont_.stateSize(), cont_.stateSize()));
}

Eigen::MatrixXd StateEstimator::makeCy()
{
  Eigen::MatrixXd Cy = Eigen::MatrixXd::Zero(kf_.outputSize(), cont_.stateSize());

  // Euler angles.
  Cy(kRollIdx, LinearDynamics::kRollIdx) = 1;
  Cy(kPitchIdx, LinearDynamics::kPitchIdx) = 1;

  // Rotational velocity.
  Cy.block<3, 3>(kGyroIdx, LinearDynamics::kGyroXIdx).diagonal().setOnes();

  // Gravity.
  Cy(kGravIdx, LinearDynamics::kGravIdx) = 1;

  // Height from the ground.
  for (size_t l = 0; l < nc_; ++l) {
    Cy(altIdx(l), LinearDynamics::kAltIdx) = 1;
  }

  // Translational velocity.
  for (size_t l = 0; l < nc_; ++l) {
    Cy.block<3, 3>(velIdx(l), LinearDynamics::kVelXIdx).diagonal().setOnes();
  }

  return Cy;
}
}  // namespace lr_tools
}  // namespace tobas
