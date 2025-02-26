#include <tobas_yaml_tools/core.hpp>

#include "tobas_drone_core/propulsion_system/ice_propulsion_system/engine.hpp"

using namespace std;

namespace tobas
{
bool EngineConfig::isValid() const
{
  if (torque_const <= 0.)
  {
    cerr << "Engine torque constant must be positive." << endl;
    return false;
  }

  if (friction_torque <= 0.)
  {
    cerr << "Engine dynamic friction torque must be positive." << endl;
    return false;
  }

  return true;
}

bool EngineConfig::load(const YAML::Node& node)
{
  if (!yaml::load(kTorqueConstantKey, node, torque_const))
    return false;

  if (!yaml::load(kDynamicFrictionTorqueKey, node, friction_torque))
    return false;

  return true;
}

YAML::Node EngineConfig::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[kTorqueConstantKey] = torque_const;
  node[kDynamicFrictionTorqueKey] = friction_torque;

  return node;
}
}  // namespace tobas
