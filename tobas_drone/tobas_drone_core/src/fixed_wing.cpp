#include <tobas_yaml_tools/core.hpp>

#include "../include/tobas_drone_core/fixed_wing.hpp"

using namespace std;

namespace tobas
{
bool FixedWingConfig::isValid() const
{
  if (!equipped)
    return true;

  if (!vehicle.isValid())
  {
    cerr << "The configurations of vehicle parameters are invalid." << endl;
    return false;
  }

  if (!aerodynamics.isValid())
  {
    cerr << "The configurations of aerodynamic parameters are invalid." << endl;
    return false;
  }

  for (const auto& [_, cs] : control_surfaces)
  {
    if (!cs.isValid())
    {
      cerr << "The configurations of control surface \"" << cs.link_name << "\" are invalid." << endl;
      return false;
    }
  }

  return true;
}

bool FixedWingConfig::load(const YAML::Node& node)
{
  // Equipped
  if (!yaml::load(kEquippedKey, node, equipped))
    return false;

  // Vehicle
  if (!node[kVehicleKey].IsMap())
  {
    cerr << "Vehicle parameters field is not defined." << endl;
    return false;
  }
  if (!vehicle.load(node[kVehicleKey]))
  {
    cerr << "Failed to load vehicle parameters." << endl;
    return false;
  }

  // Aerodynamics
  if (!node[kAerodynamicsKey].IsMap())
  {
    cerr << "Aerodynamics parameters field is not defined." << endl;
    return false;
  }
  if (!aerodynamics.load(node[kAerodynamicsKey]))
  {
    cerr << "Failed to load aerodynamic parameters." << endl;
    return false;
  }

  // Control surfaces
  control_surfaces.clear();
  if (!node[kControlSurfacesKey].IsSequence())
  {
    cerr << "Control surface field is not defined." << endl;
    return false;
  }
  for (const auto& cs_node : node[kControlSurfacesKey])
  {
    ControlSurface cs;
    if (!cs.load(cs_node))
    {
      cerr << "Failed to load the configurations of control surfaces." << endl;
      return false;
    }
    control_surfaces[cs.channel] = cs;
  }

  return true;
}

YAML::Node FixedWingConfig::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  // Equipped
  node[kEquippedKey] = equipped;

  // Vehicle
  node[kVehicleKey] = vehicle.dump();

  // Aerodynamics
  node[kAerodynamicsKey] = aerodynamics.dump();

  // Control surfaces
  node[kControlSurfacesKey] = YAML::Node(YAML::NodeType::Sequence);
  for (auto& [_, cs] : control_surfaces)
    node[kControlSurfacesKey].push_back(cs.dump());

  return node;
}
}  // namespace tobas
