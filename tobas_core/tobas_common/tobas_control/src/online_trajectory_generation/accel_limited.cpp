#include "tobas_control/online_trajectory_generation/accel_limited.hpp"

#include <algorithm>
#include <cassert>

#include <tobas_math/core.hpp>

namespace ctrl
{
AccelLimitedOnlineTrajectoryGenerator::AccelLimitedOnlineTrajectoryGenerator()
{
}

void AccelLimitedOnlineTrajectoryGenerator::setTargetPosition(double tar_pos)
{
  if (tar_pos == pf_) {
    return;
  }

  pf_ = tar_pos;

  sign_ = controlSign();
  state_ = kFirstBang;
}

void AccelLimitedOnlineTrajectoryGenerator::setTargetVelocity(double tar_vel)
{
  if (tar_vel == vf_) {
    return;
  }

  vf_ = tar_vel;

  sign_ = controlSign();
  state_ = kFirstBang;
}

void AccelLimitedOnlineTrajectoryGenerator::setMaxVelocity(double max_vel)
{
  assert(max_vel > 0.);
  vm_ = max_vel;
}

void AccelLimitedOnlineTrajectoryGenerator::setMaxAccel(double max_acc)
{
  assert(max_acc > 0.);
  am_ = max_acc;
}

void AccelLimitedOnlineTrajectoryGenerator::update(double dt)
{
  assert(std::isfinite(vm_));
  assert(std::isfinite(am_));

  switch (state_) {
    case kFirstBang:
      step(dt);
      if (isCloseToTarget(dt)) {
        fixToTarget();
        state_ = kDone;
      }
      else if (hasCrossedSwitchingCurve()) {
        state_ = kSecondBang;
      }
      break;
    case kSecondBang:
      step(dt);
      if (isCloseToTarget(dt)) {
        fixToTarget();
        state_ = kDone;
      }
      else if (hasCrossedSwitchingCurve()) {  // 理論上は2度スイッチング曲線を跨ぐことはない
        fixToTarget();
        state_ = kDone;
      }
      break;
    case kDone:
      break;
    default:
      throw;
  }
}

void AccelLimitedOnlineTrajectoryGenerator::resetCurrentTrajectoryPoint(double pos, double vel)
{
  p_ = pos;
  v_ = vel;

  sign_ = controlSign();
  state_ = kFirstBang;
}

double AccelLimitedOnlineTrajectoryGenerator::switchingCurve() const
{
  return (pf_ - p_) + std::abs(vf_ - v_) * (vf_ + v_) / (2 * am_);
}

int AccelLimitedOnlineTrajectoryGenerator::controlSign() const
{
  return math::sign(switchingCurve());
}

bool AccelLimitedOnlineTrajectoryGenerator::hasCrossedSwitchingCurve() const
{
  return sign_ * controlSign() < 0;
}

bool AccelLimitedOnlineTrajectoryGenerator::isCloseToTarget(double dt)
{
  // 速度の誤差のオーダーは O(am dt)
  // 位置の誤差のオーダーは O(am dt^2)
  return std::abs(pf_ - p_) < am_ * math::sqr(dt) && std::abs(vf_ - v_) < am_ * dt;
}

void AccelLimitedOnlineTrajectoryGenerator::fixToTarget()
{
  p_ = pf_;
  v_ = vf_;
}

void AccelLimitedOnlineTrajectoryGenerator::step(double dt)
{
  const auto a = am_ * sign_;

  // 速度（状態方程式の状態）に対する不等式制約があるため，軌跡を解析的に時間の関数で表現するのは難しい．
  // そのため数値的に軌道上の点を更新する．
  const auto next_v = std::clamp(v_ + a * dt, -vm_, vm_);
  const auto next_p = p_ + ((v_ + next_v) / 2) * dt;

  v_ = next_v;
  p_ = next_p;
}
}  // namespace ctrl
