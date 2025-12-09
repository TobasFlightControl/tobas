#include "tobas_drone_core/propulsion_system/ice_propulsion_system/drag_constant.hpp"

#include <iostream>

#include <tobas_yaml_tools/core.hpp>

using namespace std;

namespace tobas
{
bool VppDragConstant::isValid() const
{
  if (c1 <= 0.) {
    cerr << "The second term of the drag constant must be positive." << endl;
    return false;
  }

  return true;
}

bool VppDragConstant::load(const YAML::Node& node)
{
  if (!yaml::load(kC0Key, node, c0)) {
    return false;
  }

  if (!yaml::load(kC1Key, node, c1)) {
    return false;
  }

  return true;
}

YAML::Node VppDragConstant::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[kC0Key] = c0;
  node[kC1Key] = c1;

  return node;
}
}  // namespace tobas
