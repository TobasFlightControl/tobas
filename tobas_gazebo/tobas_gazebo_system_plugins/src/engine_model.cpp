#include <tobas_math/core.hpp>

#include "../include/tobas_gazebo_system_plugins/engine_model.hpp"
#include "../include/tobas_gazebo_system_plugins/sdf.hpp"

using namespace std;

namespace gazebo
{
EngineModel::EngineModel(const ICERotorModelMap& rotors) : rotors_(rotors)
{
}

bool EngineModel::initialize(const sdf::ElementConstPtr& sdf)
{
  if (!getSdfParams(sdf))
    return false;

  if (!speed_filter_.initialize(time_const_up_, time_const_down_, 0.))
    return false;

  return true;
}

double EngineModel::getTorqueConst() const
{
  return torque_const_;
}

double EngineModel::getFrictionTorque() const
{
  return friction_torque_;
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
  if (dt <= 0.)
    return false;

  position_ += getSpeed() * dt;

  const auto steady_speed = computeSteadySpeed();
  if (!speed_filter_.update(steady_speed, dt))
    return false;

  return true;
}

bool EngineModel::getSdfParams(const sdf::ElementConstPtr& sdf)
{
  if (!getSdfParam(sdf, "torqueConstant", torque_const_))
    return false;
  if (torque_const_ <= 0.)
  {
    gzerr << "Engine torque constant must be positive." << endl;
    return false;
  }

  if (!getSdfParam(sdf, "dynamicFrictionTorque", friction_torque_))
    return false;
  if (friction_torque_ <= 0.)
  {
    gzerr << "Engine friction torque must be positive." << endl;
    return false;
  }

  if (!getSdfParam(sdf, "timeConstUp", time_const_up_))
    return false;
  if (!getSdfParam(sdf, "timeConstDown", time_const_down_))
    return false;
  if (time_const_up_ < 0. || time_const_down_ < 0.)
  {
    gzerr << "The time constant of engine speed convergence must be non-negative." << endl;
    return false;
  }

  return true;
}

double EngineModel::computeSteadySpeed()
{
  const auto& A = torque_const_;
  const auto& B = friction_torque_;

  double K = 0.;
  for (const auto& [_, rotor] : rotors_)
    K += rotor.getMotorConst() * rotor.getMomentConst() / math::sqr(rotor.getGearRatio());

  const auto g = math::sqr(throttle_) * (2 - throttle_);
  const auto Ag = A * g;

  const auto D = math::sqr(Ag) - 4 * K * B;
  if (D < 0.)
    return 0.;
  else
    return (Ag + sqrt(D)) / (2 * K);
}
}  // namespace gazebo
