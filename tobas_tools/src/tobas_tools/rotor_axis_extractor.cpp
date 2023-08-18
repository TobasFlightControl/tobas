#include "../../include/tobas_tools/rotor_axis_extractor.hpp"
#include "../../include/tobas_tools/constants.hpp"

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

const uint32_t& RotorAxisExtractor::rotorIdx(uint32_t inner_idx) const
{
  return rotor_idxs_[inner_idx];
}

const std::string& RotorAxisExtractor::linkName(uint32_t inner_idx) const
{
  return drone_.rotorConfig(rotorIdx(inner_idx)).link_name;
}

const Axis& RotorAxisExtractor::axis(uint32_t inner_idx) const
{
  return drone_.rotorConfig(rotorIdx(inner_idx)).axis;
}

const int& RotorAxisExtractor::direction(uint32_t inner_idx) const
{
  return drone_.rotorConfig(rotorIdx(inner_idx)).direction;
}

const double& RotorAxisExtractor::motorConstant(uint32_t inner_idx) const
{
  return drone_.rotorConfig(rotorIdx(inner_idx)).motor_constant;
}

const double& RotorAxisExtractor::momentConstant(uint32_t inner_idx) const
{
  return drone_.rotorConfig(rotorIdx(inner_idx)).moment_constant;
}

const pair<double, double>& RotorAxisExtractor::rotSpeedCoefs(uint32_t inner_idx) const
{
  return drone_.rotorConfig(rotorIdx(inner_idx)).rot_speed_coefs;
}

double RotorAxisExtractor::thrustFromVoltage(uint32_t inner_idx, double voltage) const
{
  return drone_.thrustFromVoltage(rotorIdx(inner_idx), voltage);
}

double RotorAxisExtractor::voltageFromRotSpeed(uint32_t inner_idx, double rot_speed) const
{
  return drone_.voltageFromRotSpeed(rotorIdx(inner_idx), rot_speed);
}

double RotorAxisExtractor::rotSpeedFromVoltage(uint32_t inner_idx, double voltage) const
{
  return drone_.rotSpeedFromVoltage(rotorIdx(inner_idx), voltage);
}

double RotorAxisExtractor::rotSpeedFromThrust(uint32_t inner_idx, double thrust) const
{
  return drone_.rotSpeedFromThrust(rotorIdx(inner_idx), thrust);
}

double RotorAxisExtractor::maxThrustSum(double battery_voltage) const
{
  double res = 0.;
  for (const auto& rotor_idx : rotor_idxs_)
  {
    res += drone_.thrustFromVoltage(rotor_idx, battery_voltage);
  }
  return res;
}

double RotorAxisExtractor::minThrustSum(double battery_voltage) const
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
