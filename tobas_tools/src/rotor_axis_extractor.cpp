#include "../include/tobas_tools/rotor_axis_extractor.hpp"
#include "../include/tobas_tools/constants.hpp"

using namespace std;

namespace tobas
{
RotorAxisExtractor::RotorAxisExtractor(const Drone& drone, Axis axis) : drone_(drone), axis_(axis)
{
  if (drone.isLoaded())
    updateInternalDataStructures();
}

void RotorAxisExtractor::updateInternalDataStructures()
{
  count_ = 0;
  rotor_idxs_.clear();

  for (size_t i = 0; i < drone_.numRotors(); ++i)
  {
    if (drone_.rotorConfig(i).axis == axis_)
    {
      ++count_;
      rotor_idxs_.emplace_back(i);
    }
  }
}

double RotorAxisExtractor::thrustSum(const vector<double>& rot_speeds) const
{
  assert(rot_speeds.size() == drone_.numRotors());

  double res = 0.;
  for (const auto& rotor_idx : rotor_idxs_)
    res += drone_.thrustFromRotSpeed(rotor_idx, rot_speeds[rotor_idx]);
  return res;
}

double RotorAxisExtractor::maxThrustSum(const double& battery_voltage) const
{
  double res = 0.;
  for (const auto& rotor_idx : rotor_idxs_)
    res += drone_.thrustFromVoltage(rotor_idx, battery_voltage);
  return res;
}

double RotorAxisExtractor::minThrustSum(const double& battery_voltage) const
{
  const auto min_voltage = battery_voltage * kArmThrottle;
  double res = 0.;
  for (const auto& rotor_idx : rotor_idxs_)
    res += drone_.thrustFromVoltage(rotor_idx, min_voltage);
  return res;
}
}  // namespace tobas
