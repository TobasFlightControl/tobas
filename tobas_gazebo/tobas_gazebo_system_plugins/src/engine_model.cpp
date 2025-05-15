#include <tobas_math/core.hpp>

#include "tobas_gazebo_system_plugins/engine_model.hpp"
#include "tobas_gazebo_system_plugins/sdf.hpp"

using namespace std;

namespace gazebo
{
EngineModel::EngineModel(const ICERotorModelMap& rotors) : rotors_(rotors)
{
}

bool EngineModel::initialize(const sdf::ElementConstPtr& sdf)
{
  if (!getSdfParams(sdf)) {
    return false;
  }

  if (!speed_filter_.initialize(time_const_up_, time_const_down_, 0.)) {
    return false;
  }

  newton_.initialize(
    bind(&self::speedFunc, this, std::placeholders::_1), bind(&self::speedFuncDeriv, this, std::placeholders::_1));

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

void EngineModel::setThrottle(const double& throttle)
{
  throttle_ = clamp(throttle, 0., 1.);
}

bool EngineModel::step(const double& dt)
{
  if (dt <= 0.) {
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
  if (engine_const_.first <= 0. || engine_const_.second <= 0.) {
    gzerr << "Engine constants must be positive." << endl;
    return false;
  }

  if (!getSdfParam(sdf, "timeConstUp", time_const_up_)) {
    return false;
  }
  if (!getSdfParam(sdf, "timeConstDown", time_const_down_)) {
    return false;
  }
  if (time_const_up_ < 0. || time_const_down_ < 0.) {
    gzerr << "The time constant of engine speed convergence must be non-negative." << endl;
    return false;
  }

  return true;
}

double EngineModel::computeSteadySpeed()
{
  // FIXME: 実際はゼロスロットルでもトルクは発生する (アイドリング)
  if (throttle_ <= std::numeric_limits<double>::epsilon()) {
    return 0.;
  }

  double speed = 0.;
  if (newton_.solve(speed) < 0) {
    gzerr << "Failed to solve engine dynamics equation: " << newton_.errorMessage() << endl;
    return 0.;
  }

  return speed;
}

double EngineModel::speedFunc(double omega) const
{
  const auto& B = engine_const_.second;
  const auto f = calc_f();
  const auto k = calc_k();
  return f * math::sqr(k) * math::quat(omega) + k * omega - B;
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
  return math::sqr(A / (1 - cos(phi)));
}

double EngineModel::calc_k() const
{
  double res = 0.;
  for (const auto& [_, rotor] : rotors_) {
    res += rotor.getMotorConst() * rotor.getMomentConst() / math::cube(rotor.getGearRatio());
  }
  return res;
}
}  // namespace gazebo
