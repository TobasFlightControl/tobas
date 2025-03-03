#include <iostream>

#include "tobas_drone_core/propulsion_system/ice_propulsion_system/ice_propulsion_system.hpp"

using namespace std;

namespace tobas
{
bool ICEPropulsionSystemConfig::isValid() const
{
  // Rotors
  for (const auto& [_, rotor] : rotors)
  {
    if (!rotor->isValid())
    {
      cerr << "The configurations of rotor \"" << rotor->link_name << "\" are invalid." << endl;
      return false;
    }
  }

  // Engine
  if (!engine.isValid())
    return false;

  return true;
}

bool ICEPropulsionSystemConfig::load(const YAML::Node& node)
{
  // Rotors
  rotors.clear();
  if (!node[kRotorsKey].IsSequence())
  {
    cerr << "Rotors field is not defined." << endl;
    return false;
  }
  for (const auto& rotor_node : node[kRotorsKey])
  {
    const auto rotor = make_shared<ICERotorConfig>();
    if (!rotor->load(rotor_node))
    {
      cerr << "Failed to load the configurations of rotors." << endl;
      return false;
    }
    rotors[rotor->link_name] = rotor;
  }

  // Engine
  if (!node[kEngineKey].IsDefined())
  {
    cerr << "Engine field is not defined." << endl;
    return false;
  }
  if (!engine.load(node[kEngineKey]))
  {
    cerr << "Failed to load the configurations of engine." << endl;
    return false;
  }

  return true;
}

YAML::Node ICEPropulsionSystemConfig::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  // Rotors
  node[kRotorsKey] = YAML::Node(YAML::NodeType::Sequence);
  for (const auto& [_, rotor] : rotors)
    node[kRotorsKey].push_back(rotor->dump());

  // Engine
  node[kEngineKey] = engine.dump();

  return node;
}

propulsion_system_t ICEPropulsionSystemConfig::type() const
{
  return propulsion_system_t::ICE;
}

double ICEPropulsionSystemConfig::minSpeed(const std::string&) const
{
  return 0.;  // TODO
}

double ICEPropulsionSystemConfig::maxSpeed(const std::string&) const
{
  return 500.;  // TODO
}

double ICEPropulsionSystemConfig::minThrust(const std::string&) const
{
  return 0.;  // TODO
}

double ICEPropulsionSystemConfig::maxThrust(const std::string&) const
{
  return numeric_limits<double>::max();  // TODO
}
}  // namespace tobas
