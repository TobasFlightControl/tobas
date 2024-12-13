#include <cmath>
#include <algorithm>
#include <cassert>

#include <tobas_math/core.hpp>

#include "../include/tobas_control/online_trajectory_filter.hpp"

using namespace std;

namespace ctrl
{
inline static double sign(double v)
{
  return signbit(v) ? -1. : 1.;
}

inline static double delta_v(double v, double edk, double eddk)
{
  return eddk * fabs(eddk) + 2 * (edk - v);
}

inline static double u_cv(double v, double U, double edk, double eddk)
{
  return -U * sign(delta_v(v, edk, eddk) + (1 - fabs(sign(delta_v(v, edk, eddk)))) * eddk);
}

inline static double u_a(double a, double U, double eddk)
{
  return -U * sign(eddk - a);
}

inline static double u_v(double v, double U, double edd_min, double edd_max, double edk, double eddk)
{
  const auto min = fmin(u_cv(v, U, edk, eddk), u_a(edd_max, U, eddk));
  return fmax(u_a(edd_min, U, eddk), min);
}

OnlineTrajectoryFilter::OnlineTrajectoryFilter()
{
}

void OnlineTrajectoryFilter::setTargetPosition(double tar_pos)
{
  tar_pos_ = tar_pos;
}

void OnlineTrajectoryFilter::setTargetVelocity(double tar_vel)
{
  tar_vel_ = tar_vel;
}

void OnlineTrajectoryFilter::setTargetAcceleration(double tar_acc)
{
  tar_acc_ = tar_acc;
}

void OnlineTrajectoryFilter::setMinVelocity(double min_vel)
{
  min_vel_ = min_vel;
}

void OnlineTrajectoryFilter::setMaxVelocity(double max_vel)
{
  max_vel_ = max_vel;
}

void OnlineTrajectoryFilter::setMinAcceleration(double min_acc)
{
  min_acc_ = min_acc;
}

void OnlineTrajectoryFilter::setMaxAcceleration(double max_acc)
{
  max_acc_ = max_acc;
}

void OnlineTrajectoryFilter::setMaxJerk(double max_jerk)
{
  max_jerk_ = max_jerk;
}

void OnlineTrajectoryFilter::setSpeedOverride(double speed_override)
{
  speed_override_ = clamp(speed_override, 1e-3, 1.);
}

void OnlineTrajectoryFilter::update(
  double dt,
  double cur_pos,
  double cur_vel,
  double cur_acc,
  double& cmd_pos,
  double& cmd_vel,
  double& cmd_acc) const
{
  assert(dt >= 0.);

  // Compute control action
  const auto U = max_jerk_ * speed_override_;
  const auto v_min = min_vel_ * speed_override_;
  const auto v_max = max_vel_ * speed_override_;
  const auto a_min = min_acc_ * speed_override_;
  const auto a_max = max_acc_ * speed_override_;

  const auto ek = (cur_pos - tar_pos_) / U;
  const auto edk = (cur_vel - tar_vel_) / U;
  const auto eddk = (cur_acc - tar_acc_) / U;

  const auto ed_min = (v_min - tar_vel_) / U;
  const auto ed_max = (v_max - tar_vel_) / U;
  const auto edd_min = (a_min - tar_acc_) / U;
  const auto edd_max = (a_max - tar_acc_) / U;

  const auto delta = edk + (eddk * fabs(eddk)) / 2;
  const auto sgnd = sign(delta);

  double Sigma = 0.;
  if ((eddk <= edd_max) && (edk <= (math::sqr(eddk) / 2 - math::sqr(edd_max))))
  {
    const auto tmp = math::sqr(eddk) - 2 * edk;
    Sigma = ek - edd_max * tmp / 4 - math::sqr(tmp) / (8 * edd_max) - eddk * (3 * edk - math::sqr(eddk)) / 3;
  }
  else if ((eddk >= edd_min) && (edk >= (math::sqr(edd_min) - math::sqr(eddk) / 2)))
  {
    const auto tmp = math::sqr(eddk) + 2 * edk;
    Sigma = ek - edd_min * tmp / 4. - math::sqr(tmp) / (8 * edd_min) + eddk * (3 * edk + math::sqr(eddk)) / 3;
  }
  else
  {
    const auto tmp = math::sqr(eddk) + 2 * edk * sgnd;
    Sigma = ek + edk * eddk * sgnd - math::cube(eddk) / 6 * (1 - 3 * fabs(sgnd)) + sgnd / 4 * sqrt(2 * math::cube(tmp));
  }

  const auto uc = -U * sign(Sigma + (1 - fabs(sign(Sigma))) * (delta + (1 - fabs(sgnd) * eddk)));
  const auto min = fmin(uc, u_v(ed_max, U, edd_min, edd_max, edk, eddk));
  const auto uk = fmax(u_v(ed_min, U, edd_min, edd_max, edk, eddk), min);

  // Compute filter output
  cmd_acc = cur_acc + dt * uk;
  cmd_vel = cur_vel + dt / 2 * (cmd_acc + cur_acc);
  cmd_pos = cur_pos + dt / 2 * (cmd_vel + cur_vel);

  // Clamp output
  cmd_vel = clamp(cmd_vel, min_vel_, max_vel_);
  cmd_acc = clamp(cmd_acc, min_acc_, max_acc_);
}
}  // namespace ctrl
