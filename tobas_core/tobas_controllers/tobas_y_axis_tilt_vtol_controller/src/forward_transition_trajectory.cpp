#include "tobas_y_axis_tilt_vtol_controller/forward_transition_trajectory.hpp"

#include <iostream>

#include <tobas_std_tools/universal_constants.hpp>
#include <tobas_math/core.hpp>
#include <tobas_math/equation.hpp>
#include <tobas_ros2_tools/time.hpp>

namespace tobas
{
namespace y_axis_tilt_vtol
{
ForwardTransitionTrajectory::ForwardTransitionTrajectory()
{
}

void ForwardTransitionTrajectory::setMaxTiltVelocity(const double& max_tilt_velocity)
{
  max_tilt_velocity_ = max_tilt_velocity;
}

void ForwardTransitionTrajectory::setMinAoA(const double& min_alpha)
{
  max_pitch_ = -min_alpha;
}

void ForwardTransitionTrajectory::setMaxPitchRate(const double& max_pitch_rate)
{
  max_pitch_rate_ = max_pitch_rate;
}

void ForwardTransitionTrajectory::setCruiseVelocity(const double& cruise_speed)
{
  cruise_speed_ = cruise_speed;
}

void ForwardTransitionTrajectory::setStartConditions(const builtin_interfaces::msg::Time& start_time, const double& start_pitch_angle, const double& start_tilt_angle, const double& start_vx)
{
  start_time_ = start_time;
  start_pitch_angle_ = start_pitch_angle;
  start_tilt_angle_ = start_tilt_angle;
  start_vx_ = start_vx;
  const auto tilt_finish_time = (kForwardAngle - start_tilt_angle_) / max_tilt_velocity_;
  assert(tilt_finish_time > 0);
  assert(start_vx_ < cruise_speed_);

  const auto max_rate = max_tilt_velocity_ + max_pitch_rate_;
  const auto [t1, t2] = math::solveQuadraticEquation(
    0.5 * st::kGravity * max_rate,
    - st::kGravity * max_rate * tilt_finish_time,
    cruise_speed_ - start_vx_ - st::kGravity * (start_pitch_angle + start_tilt_angle) * tilt_finish_time);
  if (std::abs(t1.imag()) <= std::numeric_limits<double>::epsilon()) {
    const auto t1_candidate = std::min(t1.real(), t2.real());
    if (t1_candidate * max_pitch_rate_ < max_pitch_) {
      mode_ = AccelerationMode::STOP_BEFORE_PITCH_LIMIT;
      stop_before_pitch_limit_config_.tc = t1_candidate;
      stop_before_pitch_limit_config_.duration = tilt_finish_time;
      assert(stop_before_pitch_limit_config_.tc <= tilt_finish_time);
      return;
    } else {
      const auto tc1 = (max_pitch_ - start_pitch_angle) / max_pitch_rate_;
      const auto [tc2_1, tc2_2] = math::solveQuadraticEquation(
        0.5 * st::kGravity * max_tilt_velocity_,
        - st::kGravity * max_tilt_velocity_ * tilt_finish_time,
        cruise_speed_ - start_vx_ - st::kGravity * (max_pitch_ + start_tilt_angle_) * (tilt_finish_time - tc1)
          - st::kGravity * (start_pitch_angle + start_tilt_angle) * tc1
          - 0.5 * st::kGravity * max_pitch_rate_ * math::sqr(tc1)
      );
      if (std::abs(tc2_1.imag()) <= std::numeric_limits<double>::epsilon()) {
        mode_ = AccelerationMode::STOP_AT_PITCH_LIMIT;
        stop_at_pitch_limit_config_.tc1 = tc1;
        stop_at_pitch_limit_config_.tc2 = std::min(tc2_1.real(), tc2_2.real());
        stop_at_pitch_limit_config_.duration = tilt_finish_time;
        assert(stop_at_pitch_limit_config_.tc1 <= stop_at_pitch_limit_config_.tc2);
        assert(stop_at_pitch_limit_config_.tc2 <= tilt_finish_time);
        return;
      }
    }
  }
  // 加速度増加を最後まで止めない場合
  const auto tc1 = (max_pitch_ - start_pitch_angle) / max_pitch_rate_;
  const auto dV1 = st::kGravity * (start_pitch_angle + start_tilt_angle) * tc1 + 0.5 * st::kGravity * (max_tilt_velocity_ + max_pitch_rate_) * math::sqr(tc1);
  if (dV1 > cruise_speed_ - start_vx_) {
    mode_ = AccelerationMode::CONTINUE_BEFORE_PITCH_LIMIT;
    const auto [duration_1, duration_2] = math::solveQuadraticEquation(
      0.5 * st::kGravity * (max_tilt_velocity_ + max_pitch_rate_),
      st::kGravity * (start_pitch_angle + start_tilt_angle),
      - (cruise_speed_ - start_vx_)
    ); // 正と負の解が出る
    continue_before_pitch_limit_config_.duration = std::max(duration_1.real(), duration_2.real());
    continue_before_pitch_limit_config_.tilt_finish_time = tilt_finish_time;
    assert(continue_before_pitch_limit_config_.duration >= tilt_finish_time);
    return;
  } else {
    mode_ = AccelerationMode::CONTINUE_AFTER_PITCH_LIMIT;
    continue_after_pitch_limit_config_.tc = tc1;
    const auto dV2 = cruise_speed_ - start_vx_ - dV1;
    const auto [duration_1, duration_2] = math::solveQuadraticEquation(
      0.5 * st::kGravity * max_tilt_velocity_,
      st::kGravity * (max_pitch_ + start_tilt_angle_),
      -dV2
    ); // 正と負の解が出る
    continue_after_pitch_limit_config_.duration = std::max(duration_1.real(), duration_2.real()) + tc1;
    continue_after_pitch_limit_config_.tilt_finish_time = tilt_finish_time;
    assert(continue_after_pitch_limit_config_.duration >= tilt_finish_time);
    return;
  }
}

double ForwardTransitionTrajectory::calcTargetTiltAngle(const builtin_interfaces::msg::Time& cur_time)
{
  const auto elapsed_time = (cur_time - start_time_).seconds();
  return std::min(start_tilt_angle_ + max_tilt_velocity_ * elapsed_time, kForwardAngle);
}

double ForwardTransitionTrajectory::calcTargetVx(const builtin_interfaces::msg::Time& cur_time)
{
  const auto elapsed_time = (cur_time - start_time_).seconds();
  switch (mode_)
  {
  case AccelerationMode::STOP_BEFORE_PITCH_LIMIT:
    return calcTargetVxStopBeforePitchLimit(elapsed_time);
  case AccelerationMode::STOP_AT_PITCH_LIMIT:
    return calcTargetVxStopAtPitchLimit(elapsed_time);
  case AccelerationMode::CONTINUE_BEFORE_PITCH_LIMIT:
    return calcTargetVxContinueBeforePitchLimit(elapsed_time);
  case AccelerationMode::CONTINUE_AFTER_PITCH_LIMIT:
    return calcTargetVxContinueAfterPitchLimit(elapsed_time);
  default:
    std::cerr << "Unknown mode" << std::endl;
    return 0.0;
  }
}

double ForwardTransitionTrajectory::calcTargetAccx(const builtin_interfaces::msg::Time& cur_time)
{
  const auto elapsed_time = (cur_time - start_time_).seconds();
  switch (mode_)
  {
  case AccelerationMode::STOP_BEFORE_PITCH_LIMIT:
    return calcTargetAccxStopBeforePitchLimit(elapsed_time);
  case AccelerationMode::STOP_AT_PITCH_LIMIT:
    return calcTargetAccxStopAtPitchLimit(elapsed_time);
  case AccelerationMode::CONTINUE_BEFORE_PITCH_LIMIT:
    return calcTargetAccxContinueBeforePitchLimit(elapsed_time);
  case AccelerationMode::CONTINUE_AFTER_PITCH_LIMIT:
    return calcTargetAccxContinueAfterPitchLimit(elapsed_time);
  default:
    std::cerr << "Unknown mode" << std::endl;
    return 0.0;
  }
}

double ForwardTransitionTrajectory::calcTargetVxStopBeforePitchLimit(const double& elapsed_time) const
{
  if (elapsed_time < stop_before_pitch_limit_config_.tc) {
    return start_vx_
      + st::kGravity * (start_pitch_angle_ + start_tilt_angle_) * elapsed_time
      + 0.5 * st::kGravity * (max_tilt_velocity_ + max_pitch_rate_) * math::sqr(elapsed_time);
  } else if (elapsed_time < stop_before_pitch_limit_config_.duration) {
    const auto vc = start_vx_
      + st::kGravity * (start_pitch_angle_ + start_tilt_angle_) * stop_before_pitch_limit_config_.tc
      + 0.5 * st::kGravity * (max_tilt_velocity_ + max_pitch_rate_) * math::sqr(stop_before_pitch_limit_config_.tc);
    const auto ac = calcTargetAccxStopBeforePitchLimit(stop_before_pitch_limit_config_.tc); // constant
    return vc + ac * (elapsed_time - stop_before_pitch_limit_config_.tc);
  } else {
    return cruise_speed_;
  }
}

double ForwardTransitionTrajectory::calcTargetVxStopAtPitchLimit(const double& elapsed_time) const
{
  if (elapsed_time < stop_at_pitch_limit_config_.tc1) {
    return start_vx_
      + st::kGravity * (start_pitch_angle_ + start_tilt_angle_) * elapsed_time
      + 0.5 * st::kGravity * (max_tilt_velocity_ + max_pitch_rate_) * math::sqr(elapsed_time);
  } else if (elapsed_time < stop_at_pitch_limit_config_.tc2) {
    const auto tc1 = stop_at_pitch_limit_config_.tc1;
    const auto vc1 = start_vx_
      + st::kGravity * (start_pitch_angle_ + start_tilt_angle_) * tc1
      + 0.5 * st::kGravity * (max_tilt_velocity_ + max_pitch_rate_) * math::sqr(tc1);
    return vc1 + st::kGravity * (max_pitch_ + start_tilt_angle_) * (elapsed_time - tc1) + 0.5 * st::kGravity * max_tilt_velocity_ * (math::sqr(elapsed_time) - math::sqr(tc1));
  } else if (elapsed_time < stop_at_pitch_limit_config_.duration) {
    const auto tc1 = stop_at_pitch_limit_config_.tc1;
    const auto tc2 = stop_at_pitch_limit_config_.tc2;
    const auto vc1 = start_vx_
      + st::kGravity * (start_pitch_angle_ + start_tilt_angle_) * tc1
      + 0.5 * st::kGravity * (max_tilt_velocity_ + max_pitch_rate_) * math::sqr(tc1);
    const auto vc2 = vc1 + st::kGravity * (max_pitch_ + start_tilt_angle_) * (tc2 - tc1) + 0.5 * st::kGravity * max_tilt_velocity_ * (math::sqr(tc2) - math::sqr(tc1));
    const auto ac2 = calcTargetAccxStopAtPitchLimit(elapsed_time); // constant
    return vc2 + ac2 * (elapsed_time - tc2);
  } else {
    return cruise_speed_;
  }
}

double ForwardTransitionTrajectory::calcTargetVxContinueBeforePitchLimit(const double& elapsed_time) const
{
  if (elapsed_time < continue_before_pitch_limit_config_.duration) {
    return start_vx_
      + st::kGravity * (start_pitch_angle_ + start_tilt_angle_) * elapsed_time
      + 0.5 * st::kGravity * (max_tilt_velocity_ + max_pitch_rate_) * math::sqr(elapsed_time);
  } else {
    return cruise_speed_;
  }
}

double ForwardTransitionTrajectory::calcTargetVxContinueAfterPitchLimit(const double& elapsed_time) const
{
  if (elapsed_time < continue_after_pitch_limit_config_.tc) {
    return start_vx_
      + st::kGravity * (start_pitch_angle_ + start_tilt_angle_) * elapsed_time
      + 0.5 * st::kGravity * (max_tilt_velocity_ + max_pitch_rate_) * math::sqr(elapsed_time);
  } else if (elapsed_time < continue_after_pitch_limit_config_.duration) {
    const auto tc1 = continue_after_pitch_limit_config_.tc;
    const auto vc1 = start_vx_
      + st::kGravity * (start_pitch_angle_ + start_tilt_angle_) * tc1
      + 0.5 * st::kGravity * (max_tilt_velocity_ + max_pitch_rate_) * math::sqr(tc1);
    return vc1 + st::kGravity * (max_pitch_ + start_tilt_angle_) * (elapsed_time - tc1) + 0.5 * st::kGravity * max_tilt_velocity_ * (math::sqr(elapsed_time) - math::sqr(tc1));
  } else {
    return cruise_speed_;
  }
}

double ForwardTransitionTrajectory::calcTargetAccxStopBeforePitchLimit(const double& elapsed_time) const
{
  if (elapsed_time < stop_before_pitch_limit_config_.tc) {
    return st::kGravity * (start_pitch_angle_ + start_tilt_angle_ + (max_tilt_velocity_ + max_pitch_rate_) * elapsed_time); // 急激に増加
  } else if (elapsed_time < stop_before_pitch_limit_config_.duration) {
    return st::kGravity * (start_pitch_angle_ + start_tilt_angle_ + (max_tilt_velocity_ + max_pitch_rate_) * stop_before_pitch_limit_config_.tc); // constant
  } else {
    return 0.0;
  }
}

double ForwardTransitionTrajectory::calcTargetAccxStopAtPitchLimit(const double& elapsed_time) const
{
  if (elapsed_time < stop_at_pitch_limit_config_.tc1) {
    return st::kGravity * (start_pitch_angle_ + start_tilt_angle_ + (max_tilt_velocity_ + max_pitch_rate_) * elapsed_time); // 急激に増加
  }
  else if (elapsed_time < stop_at_pitch_limit_config_.tc2) {
    return st::kGravity * (max_pitch_ + start_tilt_angle_ + max_tilt_velocity_ * elapsed_time); // 緩やかに増加
  }
  else if (elapsed_time < stop_at_pitch_limit_config_.duration) {
    return st::kGravity * (max_pitch_ + start_tilt_angle_ + max_tilt_velocity_ * stop_at_pitch_limit_config_.tc2); // constant
  }
  else {
    return 0.0;
  }
}

double ForwardTransitionTrajectory::calcTargetAccxContinueBeforePitchLimit(const double& elapsed_time) const
{
  if (elapsed_time < continue_before_pitch_limit_config_.duration) {
    return st::kGravity * (start_pitch_angle_ + start_tilt_angle_ + (max_tilt_velocity_ + max_pitch_rate_) * elapsed_time); // 急激に増加
  } else {
    return 0.0;
  }
}

double ForwardTransitionTrajectory::calcTargetAccxContinueAfterPitchLimit(const double& elapsed_time) const
{
  if (elapsed_time < continue_after_pitch_limit_config_.tc) {
    return st::kGravity * (start_pitch_angle_ + start_tilt_angle_ + (max_tilt_velocity_ + max_pitch_rate_) * elapsed_time); // 急激に増加
  }
  else if (elapsed_time < continue_after_pitch_limit_config_.duration) {
    return st::kGravity * (max_pitch_ + start_tilt_angle_ + max_tilt_velocity_ * elapsed_time); // 緩やかに増加
  } else {
    return 0.0;
  }
}


} // namespace y_axis_tilt_vtol
} // namespace tobas
