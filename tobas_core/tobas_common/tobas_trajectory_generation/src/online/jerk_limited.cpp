// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_trajectory_generation/online/jerk_limited.hpp"

#include <algorithm>
#include <cassert>

#include <tobas_math/core.hpp>

namespace tobas
{
namespace traj
{
namespace
{
inline double sign(double _v)
{
  return std::signbit(_v) ? -1 : 1;
}

inline double delta_v(double _v, double _edk, double _eddk)
{
  return _eddk * std::abs(_eddk) + 2 * (_edk - _v);
}

inline double u_cv(double _v, double _u, double _edk, double _eddk)
{
  return -_u * sign(delta_v(_v, _edk, _eddk) + (1 - std::abs(sign(delta_v(_v, _edk, _eddk)))) * _eddk);
}

inline double u_a(double _a, double _u, double _eddk)
{
  return -_u * sign(_eddk - _a);
}

inline double u_v(double _v, double _u, double _edd_min, double _edd_max, double _edk, double _eddk)
{
  const auto tmp = std::min(u_cv(_v, _u, _edk, _eddk), u_a(_edd_max, _u, _eddk));
  return std::max(u_a(_edd_min, _u, _eddk), tmp);
}
}  // namespace

JerkLimitedOnlineTrajectoryGenerator::JerkLimitedOnlineTrajectoryGenerator()
{
}

void JerkLimitedOnlineTrajectoryGenerator::setMinVelocity(double _min_vel)
{
  v_min_ = _min_vel;
}

void JerkLimitedOnlineTrajectoryGenerator::setMaxVelocity(double _max_vel)
{
  v_max_ = _max_vel;
}

void JerkLimitedOnlineTrajectoryGenerator::setMinAcceleration(double _min_acc)
{
  a_min_ = _min_acc;
}

void JerkLimitedOnlineTrajectoryGenerator::setMaxAcceleration(double _max_acc)
{
  a_max_ = _max_acc;
}

void JerkLimitedOnlineTrajectoryGenerator::setMaxJerk(double _max_jerk)
{
  assert(_max_jerk > 0);
  u_ = _max_jerk;
}

void JerkLimitedOnlineTrajectoryGenerator::update(double _dt)
{
  assert(std::isfinite(v_min_));
  assert(std::isfinite(v_max_));
  assert(std::isfinite(a_min_));
  assert(std::isfinite(a_max_));
  assert(std::isfinite(u_));
  assert(v_min_ < v_max_);
  assert(a_min_ < a_max_);
  assert(u_ > 0);
  assert(_dt >= 0);

  const auto ek = (traj_pos_ - tar_pos_) / u_;
  const auto edk = (traj_vel_ - tar_vel_) / u_;
  const auto eddk = (traj_acc_ - tar_acc_) / u_;

  const auto ed_min = (v_min_ - tar_vel_) / u_;
  const auto ed_max = (v_max_ - tar_vel_) / u_;
  const auto edd_min = (a_min_ - tar_acc_) / u_;
  const auto edd_max = (a_max_ - tar_acc_) / u_;

  const auto delta = edk + (eddk * std::abs(eddk)) / 2;
  const auto sgnd = sign(delta);

  double sigma = 0;
  if ((eddk <= edd_max) && (edk <= (math::sqr(eddk) / 2 - math::sqr(edd_max)))) {
    const auto tmp = math::sqr(eddk) - 2 * edk;
    sigma = ek - edd_max * tmp / 4 - math::sqr(tmp) / (8 * edd_max) - eddk * (3 * edk - math::sqr(eddk)) / 3;
  }
  else if ((eddk >= edd_min) && (edk >= (math::sqr(edd_min) - math::sqr(eddk) / 2))) {
    const auto tmp = math::sqr(eddk) + 2 * edk;
    sigma = ek - edd_min * tmp / 4.0 - math::sqr(tmp) / (8 * edd_min) + eddk * (3 * edk + math::sqr(eddk)) / 3;
  }
  else {
    const auto tmp = math::sqr(eddk) + 2 * edk * sgnd;
    sigma = ek + edk * eddk * sgnd - math::cube(eddk) / 6 * (1 - 3 * std::abs(sgnd)) +
            sgnd / 4 * std::sqrt(2 * math::cube(tmp));
  }

  const auto uc = -u_ * sign(sigma + (1 - std::abs(sign(sigma))) * (delta + (1 - std::abs(sgnd) * eddk)));
  const auto tmp = std::min(uc, u_v(ed_max, u_, edd_min, edd_max, edk, eddk));
  const auto uk = std::max(u_v(ed_min, u_, edd_min, edd_max, edk, eddk), tmp);

  // Compute filter output.
  const auto next_acc = traj_acc_ + _dt * uk;
  const auto next_vel = traj_vel_ + _dt / 2 * (traj_acc_ + next_acc);
  const auto next_pos = traj_pos_ + _dt / 2 * (traj_vel_ + next_vel);

  // Update trajectory point.
  traj_acc_ = std::clamp(next_acc, a_min_, a_max_);
  traj_vel_ = std::clamp(next_vel, v_min_, v_max_);
  traj_pos_ = next_pos;
}

void JerkLimitedOnlineTrajectoryGenerator::setTargetPointAndUpdate(
  double _tar_pos,
  double _tar_vel,
  double _tar_acc,
  double _dt)
{
  setTargetPosition(_tar_pos);
  setTargetVelocity(_tar_vel);
  setTargetAcceleration(_tar_acc);
  update(_dt);
}

void JerkLimitedOnlineTrajectoryGenerator::resetCurrentTrajectoryPoint(double _pos, double _vel, double _acc)
{
  traj_pos_ = _pos;
  traj_vel_ = _vel;
  traj_acc_ = _acc;
}
}  // namespace traj
}  // namespace tobas
