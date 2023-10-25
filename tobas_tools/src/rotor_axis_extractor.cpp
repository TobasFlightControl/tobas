#include "../include/tobas_tools/rotor_axis_extractor.hpp"
#include "../include/tobas_tools/constants.hpp"

using namespace std;

namespace tobas
{
RotorAxisExtractor::RotorAxisExtractor(const Drone& drone, Axis axis) : drone_(drone), axis_(axis)
{
  if (drone.isLoaded())
  {
    updateInternalDataStructures();
  }
}

void RotorAxisExtractor::updateInternalDataStructures()
{
  setRotorIdxs();
}

uint32_t RotorAxisExtractor::count() const
{
  return rotor_idxs_.size();
}

const uint32_t& RotorAxisExtractor::rotorIdx(const uint32_t& inner_idx) const
{
  return rotor_idxs_[inner_idx];
}

const string& RotorAxisExtractor::linkName(const uint32_t& inner_idx) const
{
  return drone_.rotorConfig(rotorIdx(inner_idx)).link_name;
}

const Axis& RotorAxisExtractor::axis(const uint32_t& inner_idx) const
{
  return drone_.rotorConfig(rotorIdx(inner_idx)).axis;
}

const int& RotorAxisExtractor::direction(const uint32_t& inner_idx) const
{
  return drone_.rotorConfig(rotorIdx(inner_idx)).direction;
}

const double& RotorAxisExtractor::motorConstant(const uint32_t& inner_idx) const
{
  return drone_.rotorConfig(rotorIdx(inner_idx)).motor_constant;
}

const double& RotorAxisExtractor::momentConstant(const uint32_t& inner_idx) const
{
  return drone_.rotorConfig(rotorIdx(inner_idx)).moment_constant;
}

const double& RotorAxisExtractor::dragConstant(const uint32_t& inner_idx) const
{
  return drone_.rotorConfig(rotorIdx(inner_idx)).drag_constant;
}

const pair<double, double>& RotorAxisExtractor::rotSpeedCoefs(const uint32_t& inner_idx) const
{
  return drone_.rotorConfig(rotorIdx(inner_idx)).rot_speed_coefs;
}

double RotorAxisExtractor::thrustFromVoltage(const uint32_t& inner_idx, const double& voltage) const
{
  return drone_.thrustFromVoltage(rotorIdx(inner_idx), voltage);
}

double
RotorAxisExtractor::voltageFromRotSpeed(const uint32_t& inner_idx, const double& rot_speed) const
{
  return drone_.voltageFromRotSpeed(rotorIdx(inner_idx), rot_speed);
}

double
RotorAxisExtractor::rotSpeedFromVoltage(const uint32_t& inner_idx, const double& voltage) const
{
  return drone_.rotSpeedFromVoltage(rotorIdx(inner_idx), voltage);
}

double RotorAxisExtractor::rotSpeedFromThrust(const uint32_t& inner_idx, const double& thrust) const
{
  return drone_.rotSpeedFromThrust(rotorIdx(inner_idx), thrust);
}

double RotorAxisExtractor::thrustSum(const vector<double>& rot_speeds)
{
  assert(rot_speeds.size() == drone_.numRotors());

  double res = 0.;
  for (const auto& rotor_idx : rotor_idxs_)
  {
    res += drone_.thrustFromRotSpeed(rotor_idx, rot_speeds[rotor_idx]);
  }
  return res;
}

double RotorAxisExtractor::maxThrustSum(const double& battery_voltage) const
{
  double res = 0.;
  for (const auto& rotor_idx : rotor_idxs_)
  {
    res += drone_.thrustFromVoltage(rotor_idx, battery_voltage);
  }
  return res;
}

double RotorAxisExtractor::minThrustSum(const double& battery_voltage) const
{
  const auto min_voltage = battery_voltage * kMotorSpinArm;
  double res = 0.;
  for (const auto& rotor_idx : rotor_idxs_)
  {
    res += drone_.thrustFromVoltage(rotor_idx, min_voltage);
  }
  return res;
}

void RotorAxisExtractor::setRotorIdxs()
{
  rotor_idxs_.clear();

  for (uint32_t i = 0; i < drone_.numRotors(); ++i)
  {
    if (drone_.rotorConfig(i).axis == axis_)
    {
      rotor_idxs_.emplace_back(i);
    }
  }
}
}  // namespace tobas
