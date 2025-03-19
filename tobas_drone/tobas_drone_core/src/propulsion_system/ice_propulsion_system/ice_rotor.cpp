#include <tobas_yaml_tools/core.hpp>
#include <tobas_yaml_tools/convert/range.hpp>

#include "tobas_drone_core/propulsion_system/ice_propulsion_system/ice_rotor.hpp"

using namespace std;

namespace tobas
{
bool ICERotorConfig::isValid() const
{
  if (!super::isValid())
    return false;

  if (gear_ratio <= 0.)
  {
    cerr << "Gear ratio must be positive." << endl;
    return false;
  }

  if (!pitch_range.isValid())
  {
    cerr << "Pitch range is invalid." << endl;
    return false;
  }

  if (!pitch_range.inRange(pitch_ref))
  {
    cerr << "Pitch reference is out of range." << endl;
    return false;
  }

  if (motor_const.first <= 0.)
  {
    cerr << "The first term of motor constant must be positive." << endl;
  }

  return true;
}

bool ICERotorConfig::load(const YAML::Node& node)
{
  if (!super::load(node))
    return false;

  if (!yaml::load(kGearRatioKey, node, gear_ratio))
    return false;

  if (!yaml::load(kPitchReferenceKey, node, pitch_ref))
    return false;

  if (!yaml::load(kPitchRangeKey, node, pitch_range))
    return false;

  if (!yaml::load(kMotorConstKey, node, motor_const))
    return false;

  if (!yaml::load(kHardwareIfaceKey, node, hw_iface))
    return false;

  return true;
}

YAML::Node ICERotorConfig::dump() const
{
  auto node = super::dump();

  node[kGearRatioKey] = gear_ratio;
  node[kPitchReferenceKey] = pitch_ref;
  node[kPitchRangeKey] = pitch_range;
  node[kMotorConstKey] = motor_const;
  node[kHardwareIfaceKey] = hw_iface;

  return node;
}
}  // namespace tobas
