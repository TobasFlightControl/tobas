#include "tobas_control/online_trajectory_generation/jerk_limited.hpp"

#include <algorithm>
#include <cassert>

#include <tobas_math/core.hpp>

namespace ctrl
{
namespace
{
inline double sign(double v)
{
  return std::signbit(v) ? -1 : 1;
}

inline double delta_v(double v, double edk, double eddk)
{
  return eddk * std::abs(eddk) + 2 * (edk - v);
}

inline double u_cv(double v, double u, double edk, double eddk)
{
  return -u * sign(delta_v(v, edk, eddk) + (1 - std::abs(sign(delta_v(v, edk, eddk)))) * eddk);
}

inline double u_a(double a, double u, double eddk)
{
  return -u * sign(eddk - a);
}

inline double u_v(double v, double u, double edd_min, double edd_max, double edk, double eddk)
{
  const auto min = std::min(u_cv(v, u, edk, eddk), u_a(edd_max, u, eddk));
  return std::max(u_a(edd_min, u, eddk), min);
}
}  // namespace

JerkLimitedOnlineTrajectoryGenerator::JerkLimitedOnlineTrajectoryGenerator()
{
}

void JerkLimitedOnlineTrajectoryGenerator::setMinVelocity(double min_vel)
{
  v_min_ = min_vel;
}

void JerkLimitedOnlineTrajectoryGenerator::setMaxVelocity(double max_vel)
{
  v_max_ = max_vel;
}

void JerkLimitedOnlineTrajectoryGenerator::setMinAcceleration(double min_acc)
{
  a_min_ = min_acc;
}

void JerkLimitedOnlineTrajectoryGenerator::setMaxAcceleration(double max_acc)
{
  a_max_ = max_acc;
}

void JerkLimitedOnlineTrajectoryGenerator::setMaxJerk(double max_jerk)
{
  assert(max_jerk > 0);
  u_ = max_jerk;
}

void JerkLimitedOnlineTrajectoryGenerator::update(double dt, double cur_pos, double cur_vel, double cur_acc)
{
  assert(std::isfinite(v_min_));
  assert(std::isfinite(v_max_));
  assert(std::isfinite(a_min_));
  assert(std::isfinite(a_max_));
  assert(std::isfinite(u_));
  assert(v_min_ < v_max_);
  assert(a_min_ < a_max_);
  assert(u_ > 0);
  assert(dt >= 0);

  const auto ek = (cur_pos - tar_pos_) / u_;
  const auto edk = (cur_vel - tar_vel_) / u_;
  const auto eddk = (cur_acc - tar_acc_) / u_;

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
    sigma = ek - edd_min * tmp / 4. - math::sqr(tmp) / (8 * edd_min) + eddk * (3 * edk + math::sqr(eddk)) / 3;
  }
  else {
    const auto tmp = math::sqr(eddk) + 2 * edk * sgnd;
    sigma =
      ek + edk * eddk * sgnd - math::cube(eddk) / 6 * (1 - 3 * std::abs(sgnd)) + sgnd / 4 * sqrt(2 * math::cube(tmp));
  }

  const auto uc = -u_ * sign(sigma + (1 - std::abs(sign(sigma))) * (delta + (1 - std::abs(sgnd) * eddk)));
  const auto min = std::min(uc, u_v(ed_max, u_, edd_min, edd_max, edk, eddk));
  const auto uk = std::max(u_v(ed_min, u_, edd_min, edd_max, edk, eddk), min);

  // Compute filter output
  traj_acc_ = cur_acc + dt * uk;
  traj_vel_ = cur_vel + dt / 2 * (traj_acc_ + cur_acc);
  traj_pos_ = cur_pos + dt / 2 * (traj_vel_ + cur_vel);

  // Clamp output
  traj_vel_ = std::clamp(traj_vel_, v_min_, v_max_);
  traj_acc_ = std::clamp(traj_acc_, a_min_, a_max_);
}

void JerkLimitedOnlineTrajectoryGenerator::update(double dt)
{
  update(dt, traj_pos_, traj_vel_, traj_acc_);
}

void JerkLimitedOnlineTrajectoryGenerator::resetCurrentTrajectoryPoint(double pos, double vel, double acc)
{
  traj_pos_ = pos;
  traj_vel_ = vel;
  traj_acc_ = acc;
}
}  // namespace ctrl
