#include <tobas_yaml_tools/core.hpp>

#include "tobas_drone_core/propulsion_system/rotor.hpp"

using namespace std;

namespace tobas
{
bool RotorConfig::isValid() const
{
  if (link_name.empty())
  {
    cerr << "Link name is empty." << endl;
    return false;
  }

  if (moment_const <= 0)
  {
    cerr << "Moment constant must be positive." << endl;
    return false;
  }

  return true;
}

bool RotorConfig::load(const YAML::Node& node)
{
  if (!yaml::load(kLinkNameKey, node, link_name))
    return false;

  if (!yaml::load(kDirectionKey, node, direction))
    return false;

  if (!yaml::load(kAxisKey, node, axis))
    return false;

  if (!yaml::load(kMomentConstKey, node, moment_const))
    return false;

  if (!yaml::load(kTiltJointName, node, tilt_joint_name))
    return false;

  return true;
}

YAML::Node RotorConfig::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[kLinkNameKey] = link_name;
  node[kDirectionKey] = direction;
  node[kAxisKey] = axis;
  node[kMomentConstKey] = moment_const;
  node[kTiltJointName] = tilt_joint_name;

  return node;
}
}  // namespace tobas
