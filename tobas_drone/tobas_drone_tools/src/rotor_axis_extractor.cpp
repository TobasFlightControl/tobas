#include "../include/tobas_drone_tools/rotor_axis_extractor.hpp"

using namespace std;

namespace tobas
{
RotorAxisExtractor::RotorAxisExtractor(const Drone& drone, const rotor_axis_t& axis) : drone_(drone), axis_(axis)
{
  updateInternalDataStructures();
}

void RotorAxisExtractor::updateInternalDataStructures()
{
  channels_.clear();

  for (const auto& [channel, rotor] : drone_.rotors)
    if (rotor.axis == axis_)
      channels_.emplace_back(channel);
}

double RotorAxisExtractor::thrustSum(const vector<double>& rot_speeds) const
{
  assert(rot_speeds.size() == count());

  double res = 0.;
  for (size_t i = 0; i < count(); ++i)
    res += rotor(channels_.at(i)).thrustFromRotSpeed(rot_speeds.at(i));
  return res;
}

double RotorAxisExtractor::thrustSum(const double& battery_voltage, const double& throttle)
{
  assert(battery_voltage >= 0);
  assert(0 <= throttle && throttle <= 1);

  const auto input_voltage = battery_voltage * throttle;  // 印加電圧
  double res = 0.;
  for (const auto& channel : channels_)
    res += rotor(channel).thrustFromVoltage(input_voltage);
  return res;
}

double RotorAxisExtractor::maxThrustSum(const double& battery_voltage) const
{
  double res = 0.;
  for (const auto& channel : channels_)
    res += rotor(channel).maxThrust(battery_voltage);
  return res;
}

double RotorAxisExtractor::minThrustSum(const double& battery_voltage) const
{
  double res = 0.;
  for (const auto& channel : channels_)
    res += rotor(channel).minThrust(battery_voltage);
  return res;
}
}  // namespace tobas
