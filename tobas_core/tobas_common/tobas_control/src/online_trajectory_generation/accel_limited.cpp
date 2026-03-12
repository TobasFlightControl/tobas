#include "tobas_control/online_trajectory_generation/accel_limited.hpp"

#include <algorithm>
#include <cassert>
#include <iostream>

#include <tobas_math/core.hpp>

namespace ctrl
{
namespace
{
double solveQuadraticEquationPositive(double a, double b, double c)
{
  assert(a > 0);
  const auto d = math::sqr(b) - 4 * a * c;
  assert(d > 0);
  return (-b + sqrt(d)) / (2 * a);
}
}  // namespace

AccelLimitedOnlineTrajectoryGenerator::AccelLimitedOnlineTrajectoryGenerator()
{
}

void AccelLimitedOnlineTrajectoryGenerator::setMaxAccel(double max_acc)
{
  assert(max_acc > 0.);
  am_ = max_acc;
}

void AccelLimitedOnlineTrajectoryGenerator::update(double t)
{
  assert(std::isfinite(am_));
  assert(t >= 0);

  const auto td = (vf_ - v_) / am_;
  const auto am_2 = am_ / 2;

  const auto s = (pf_ - p_) + std::abs(vf_ - v_) * (vf_ + v_) / (2 * am_);

  // FIXME: スイッチング曲線上から少しだけ設定値が動いた場合に必要以上の速度が指令される
  if (s < 0) {  // -a -> +a
    const auto ts = solveQuadraticEquationPositive(am_, -2 * v_, pf_ - p_ - vf_ * td + am_2 * math::sqr(td));
    const auto tf = 2 * ts + td;
    if (t < ts) {
      p_ = p_ + v_ * t - am_2 * math::sqr(t);
      v_ = v_ - am_ * t;
    }
    else if (t < tf) {
      const auto tr = tf - t;
      p_ = pf_ - vf_ * tr + am_2 * math::sqr(tr);
      v_ = vf_ - am_ * tr;
    }
    else {
      p_ = pf_;
      v_ = vf_;
    }

    std::cout << s << ", " << ts << ", " << tf << ", " << t << std::endl;
  }
  else {  // +a -> -a
    const auto ts = solveQuadraticEquationPositive(am_, 2 * v_, p_ - pf_ - vf_ * td + am_2 * math::sqr(td));
    const auto tf = 2 * ts - td;
    if (t < ts) {
      p_ = p_ + v_ * t + am_2 * math::sqr(t);
      v_ = v_ + am_ * t;
    }
    else if (t < tf) {
      const auto tr = tf - t;
      p_ = pf_ - vf_ * tr - am_2 * math::sqr(tr);
      v_ = vf_ + am_ * tr;
    }
    else {
      p_ = pf_;
      v_ = vf_;
    }
  }
}

void AccelLimitedOnlineTrajectoryGenerator::resetCurrentTrajectoryPoint(double pos, double vel)
{
  p_ = pos;
  v_ = vel;
}
}  // namespace ctrl
