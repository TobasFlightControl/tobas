#include <tobas_yaml_tools/core.hpp>

#include "tobas_drone_core/joint/joint.hpp"

using namespace std;

namespace tobas
{
bool JointConfig::isValid() const
{
  if (name.empty())
  {
    cerr << "Joint name is empty." << endl;
    return false;
  }

  return true;
}

bool JointConfig::load(const YAML::Node& node)
{
  if (!yaml::load(kNameKey, node, name))
    return false;

  if (!yaml::load(kRoleKey, node, role))
    return false;

  if (!yaml::load(kCmdIfaceKey, node, cmd_iface))
    return false;

  if (!yaml::load(kHwIfaceKey, node, hw_iface))
    return false;

  if (!yaml::load(kHomePosKey, node, home_pos))
    return false;

  return true;
}

YAML::Node JointConfig::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[kNameKey] = name;
  node[kRoleKey] = role;
  node[kCmdIfaceKey] = cmd_iface;
  node[kHwIfaceKey] = hw_iface;
  node[kHomePosKey] = home_pos;

  return node;
}
}  // namespace tobas
