// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <tobas_math/core.hpp>

#include "tobas_gazebo_system_plugins/engine_model.hpp"
#include "tobas_gazebo_system_plugins/sdf.hpp"

namespace tobas
{
namespace gazebo
{
namespace
{
constexpr double kDefaultVibrationForceCoef = 0.0015;
constexpr double kDefaultVibrationForceVariationRate = 0.2;
constexpr double kDefaultVibrationDoubleFreqCoef = 1.0;
}  // namespace

EngineModel::EngineModel(const IceRotorModelMap& rotors) : rotors_(rotors), rnd_gen_(rnd_dev_())
{
}

bool EngineModel::initialize(const sdf::ElementConstPtr& sdf)
{
  if (!getSdfParams(sdf)) {
    return false;
  }

  if (!speed_filter_.initialize(time_const_up_, time_const_down_, 0.0)) {
    return false;
  }

  newton_.initialize(
    bind(&self::speedFunc, this, std::placeholders::_1), bind(&self::speedFuncDeriv, this, std::placeholders::_1));

  rice_ = RiceDistribution(1.0, vibration_force_variation_rate_);

  return true;
}

double EngineModel::getSpeed() const
{
  return speed_filter_.getValue();
}

double EngineModel::getPosition() const
{
  return position_;
}

double EngineModel::getVibrationForce()
{
  // Represent reciprocating inertia force with a sine wave and amplitude variation.
  // Use a Rice distribution centered at 1 because the expression `(1 + n)`, where `n` follows a normal distribution,
  // may be negative.
  // TODO: Analyze frequency-domain results from the real IMU and build a more accurate vibration model.
  const auto amp = vibration_force_coef_ * math::sqr(getSpeed());
  return amp * (std::sin(position_) + vibration_double_freq_coef_ * std::sin(position_ * 2)) * rice_(rnd_gen_);
}

void EngineModel::setThrottle(const double& throttle)
{
  throttle_ = std::clamp(throttle, 0.0, 1.0);
}

bool EngineModel::step(const double& dt)
{
  if (dt <= 0.0) {
    return false;
  }

  position_ += getSpeed() * dt;

  const auto steady_speed = computeSteadySpeed();
  if (!speed_filter_.update(steady_speed, dt)) {
    return false;
  }

  return true;
}

bool EngineModel::getSdfParams(const sdf::ElementConstPtr& sdf)
{
  if (!getSdfParam(sdf, "engineConstant", engine_const_)) {
    return false;
  }
  if (engine_const_.first <= 0.0 || engine_const_.second <= 0.0) {
    gzerr << "Engine constants must be positive." << std::endl;
    return false;
  }

  if (!getSdfParam(sdf, "timeConstUp", time_const_up_)) {
    return false;
  }
  if (!getSdfParam(sdf, "timeConstDown", time_const_down_)) {
    return false;
  }
  if (time_const_up_ < 0.0 || time_const_down_ < 0.0) {
    gzerr << "The time constant of engine speed convergence must be non-negative." << std::endl;
    return false;
  }

  getSdfParam(sdf, "vibrationForceCoefficient", vibration_force_coef_, kDefaultVibrationForceCoef);
  if (vibration_force_coef_ < 0.0) {
    gzerr << "The vibration force coefficient must be non-negative." << std::endl;
    return false;
  }

  getSdfParam(sdf, "vibrationForceVariationRate", vibration_force_variation_rate_, kDefaultVibrationForceVariationRate);
  if (vibration_force_variation_rate_ < 0.0) {
    gzerr << "The vibration force variation rate must be non-negative." << std::endl;
    return false;
  }

  getSdfParam(sdf, "vibrationDoubleFrequencyCoefficient", vibration_double_freq_coef_, kDefaultVibrationDoubleFreqCoef);
  if (vibration_double_freq_coef_ < 0.0) {
    gzerr << "The vibration double frequency coefficient must be non-negative." << std::endl;
    return false;
  }

  return true;
}

double EngineModel::computeSteadySpeed()
{
  // FIXME: Torque is actually generated even at zero throttle due to idling.
  if (throttle_ <= std::numeric_limits<double>::epsilon()) {
    return 0.0;
  }

  double speed = 0.0;
  if (newton_.solve(speed) < 0) {
    gzerr << "Failed to solve the engine dynamics equation: " << newton_.errorMessage() << std::endl;
    return 0.0;
  }

  return speed;
}

double EngineModel::speedFunc(double omega) const
{
  const auto& B = engine_const_.second;
  const auto f = calc_f();
  const auto k = calc_k();
  return f * math::sqr(k) * math::quar(omega) + k * omega - B;
}

double EngineModel::speedFuncDeriv(double omega) const
{
  const auto f = calc_f();
  const auto k = calc_k();
  return 4 * f * math::sqr(k) * math::cube(omega) + k;
}

double EngineModel::calc_phi() const
{
  return M_PI_2 * throttle_;
}

double EngineModel::calc_f() const
{
  const auto& A = engine_const_.first;
  const auto phi = calc_phi();
  return math::sqr(A / (1 - std::cos(phi)));
}

double EngineModel::calc_k() const
{
  double res = 0.0;
  for (const auto& [_, rotor] : rotors_) {
    res += rotor.getMotorConst() * rotor.getMomentConst() / math::cube(rotor.getGearRatio());
  }
  return res;
}
}  // namespace gazebo
}  // namespace tobas
