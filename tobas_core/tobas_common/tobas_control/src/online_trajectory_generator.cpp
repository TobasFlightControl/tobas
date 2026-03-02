#include "tobas_control/online_trajectory_generator.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>

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

OnlineTrajectoryGenerator::OnlineTrajectoryGenerator()
{
}

void OnlineTrajectoryGenerator::setMinVelocity(double min_vel)
{
  min_vel_ = min_vel;
}

void OnlineTrajectoryGenerator::setMaxVelocity(double max_vel)
{
  max_vel_ = max_vel;
}

void OnlineTrajectoryGenerator::setMinAcceleration(double min_acc)
{
  min_acc_ = min_acc;
}

void OnlineTrajectoryGenerator::setMaxAcceleration(double max_acc)
{
  max_acc_ = max_acc;
}

void OnlineTrajectoryGenerator::setMaxJerk(double max_jerk)
{
  assert(max_jerk > 0.);
  max_jerk_ = max_jerk;
}

void OnlineTrajectoryGenerator::setSpeedOverride(double speed_override)
{
  speed_override_ = std::clamp(speed_override, 1e-3, 1.);
}

void OnlineTrajectoryGenerator::update(double dt, double cur_pos, double cur_vel, double cur_acc)
{
  assert(std::isfinite(min_vel_));
  assert(std::isfinite(max_vel_));
  assert(std::isfinite(min_acc_));
  assert(std::isfinite(max_acc_));
  assert(std::isfinite(max_jerk_));
  assert(min_vel_ < max_vel_);
  assert(min_acc_ < max_acc_);
  assert(max_jerk_ > 0);
  assert(dt >= 0);

  // Compute control action
  const auto u = max_jerk_ * speed_override_;
  const auto v_min = min_vel_ * speed_override_;
  const auto v_max = max_vel_ * speed_override_;
  const auto a_min = min_acc_ * speed_override_;
  const auto a_max = max_acc_ * speed_override_;

  const auto ek = (cur_pos - tar_pos_) / u;
  const auto edk = (cur_vel - tar_vel_) / u;
  const auto eddk = (cur_acc - tar_acc_) / u;

  const auto ed_min = (v_min - tar_vel_) / u;
  const auto ed_max = (v_max - tar_vel_) / u;
  const auto edd_min = (a_min - tar_acc_) / u;
  const auto edd_max = (a_max - tar_acc_) / u;

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

  const auto uc = -u * sign(sigma + (1 - std::abs(sign(sigma))) * (delta + (1 - std::abs(sgnd) * eddk)));
  const auto min = std::min(uc, u_v(ed_max, u, edd_min, edd_max, edk, eddk));
  const auto uk = std::max(u_v(ed_min, u, edd_min, edd_max, edk, eddk), min);

  // Compute filter output
  traj_acc_ = cur_acc + dt * uk;
  traj_vel_ = cur_vel + dt / 2 * (traj_acc_ + cur_acc);
  traj_pos_ = cur_pos + dt / 2 * (traj_vel_ + cur_vel);

  // Clamp output
  traj_vel_ = std::clamp(traj_vel_, min_vel_, max_vel_);
  traj_acc_ = std::clamp(traj_acc_, min_acc_, max_acc_);
}

void OnlineTrajectoryGenerator::update(double dt)
{
  update(dt, traj_pos_, traj_vel_, traj_acc_);
}

void OnlineTrajectoryGenerator::resetCurrentTrajectoryPoint(double pos, double vel, double acc)
{
  traj_pos_ = pos;
  traj_vel_ = vel;
  traj_acc_ = acc;
}
}  // namespace ctrl
