#include <tobas_yaml_tools/core.hpp>

#include "../../include/tobas_drone_core/joint/joint.hpp"

using namespace std;

namespace tobas
{
bool JointConfig::isValid() const
{
  if (home_pos < min_pos || max_pos < home_pos)
  {
    cerr << "The home position is out of its limit." << endl;
    return false;
  }

  return true;
}

bool JointConfig::load(const YAML::Node& node)
{
  if (!yaml::load(kChannelKey, node, channel))
    return false;

  if (!yaml::load(kNameKey, node, name))
    return false;

  if (!yaml::load(kHomePosKey, node, home_pos))
    return false;

  if (!yaml::load(kMinPosKey, node, min_pos))
    return false;

  if (!yaml::load(kMaxPosKey, node, max_pos))
    return false;

  if (!yaml::load(kRoleKey, node, role))
    return false;

  if (!yaml::load(kCmdIfaceKey, node, cmd_iface))
    return false;

  if (!yaml::load(kHwIfaceKey, node, hw_iface))
    return false;

  return true;
}

YAML::Node JointConfig::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[kChannelKey] = channel;
  node[kNameKey] = name;
  node[kHomePosKey] = home_pos;
  node[kMinPosKey] = min_pos;
  node[kMaxPosKey] = max_pos;
  node[kRoleKey] = role;
  node[kCmdIfaceKey] = cmd_iface;
  node[kHwIfaceKey] = hw_iface;

  return node;
}
}  // namespace tobas
