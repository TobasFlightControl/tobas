#include <tobas_yaml_tools/core.hpp>

#include "../include/tobas_drone_core/drone.hpp"

using namespace std;
namespace fs = filesystem;

namespace tobas
{
bool Drone::isValid() const
{
  if (name == "")
  {
    cerr << "This drone does not have name." << endl;
    return false;
  }

  if (!battery.isValid())
    return false;

  for (const auto& [_, joint] : joints)
  {
    if (!joint.isValid())
    {
      cerr << "The configurations of joint \"" << joint.name << "\" are invalid." << endl;
      return false;
    }
  }

  for (const auto& [_, rotor] : rotors)
  {
    if (!rotor.isValid())
    {
      cerr << "The configurations of rotor \"" << rotor.link_name << "\" are invalid." << endl;
      return false;
    }
  }

  if (!fixed_wing.isValid())
  {
    cerr << "The configurations of fixed wing are invalid." << endl;
    return false;
  }

  return true;
}

bool Drone::load(const YAML::Node& node)
{
  // Name
  if (!yaml::load(kNameKey, node, name))
    return false;

  // Battery
  if (!node[kBatteryKey].IsMap())
  {
    cerr << "Battery field is not defined." << endl;
    return false;
  }
  if (!battery.load(node[kBatteryKey]))
  {
    cerr << "Failed to load the configurations of battery." << endl;
    return false;
  }

  // Joints
  joints.clear();
  if (!node[kJointsKey].IsSequence())
  {
    cerr << "Joints field is not defined." << endl;
    return false;
  }
  for (const auto& joint_node : node[kJointsKey])
  {
    JointConfig joint;
    if (!joint.load(joint_node))
    {
      cerr << "Failed to load the configurations of joints." << endl;
      return false;
    }
    joints[joint.name] = joint;
  }

  // Rotors
  rotors.clear();
  if (!node[kRotorsKey].IsSequence())
  {
    cerr << "Rotors field is not defined." << endl;
    return false;
  }
  for (const auto& rotor_node : node[kRotorsKey])
  {
    RotorConfig rotor;
    if (!rotor.load(rotor_node))
    {
      cerr << "Failed to load the configurations of rotors." << endl;
      return false;
    }
    rotors[rotor.channel] = rotor;
  }

  // Fixed wing
  if (!node[kFixedWingKey].IsMap())
  {
    cerr << "Fixed wing field is not defined." << endl;
    return false;
  }
  if (!fixed_wing.load(node[kFixedWingKey]))
  {
    cerr << "Failed to load the configurations of fixed wing." << endl;
    return false;
  }

  return true;
}

YAML::Node Drone::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  // Name
  node[kNameKey] = name;

  // Battery
  node[kBatteryKey] = battery.dump();

  // Joints
  node[kJointsKey] = YAML::Node(YAML::NodeType::Sequence);
  for (auto& [_, joint] : joints)
    node[kJointsKey].push_back(joint.dump());

  // Rotors
  node[kRotorsKey] = YAML::Node(YAML::NodeType::Sequence);
  for (auto& [_, rotor] : rotors)
    node[kRotorsKey].push_back(rotor.dump());

  // Fixed wing
  node[kFixedWingKey] = fixed_wing.dump();

  return node;
}

bool Drone::load(const fs::path& path)
{
  if (path.extension() != kDroneExt)
  {
    cerr << "Invalid extension: " << path.extension() << endl;
    return false;
  }

  YAML::Node node;
  if (!yaml::load(path, node))
    return false;

  if (!load(node))
  {
    cerr << "Failed to load drone." << endl;
    return false;
  }

  return true;
}

bool Drone::save(const fs::path& path) const
{
  if (path.extension() != kDroneExt)
  {
    cerr << "Invalid extension: " << path.extension() << endl;
    return false;
  }

  const auto node = dump();

  if (!yaml::save(path, node))
    return false;

  return true;
}

bool Drone::hasServoJoint() const
{
  for (const auto& [_, joint] : joints)
    if (joint.isServoJoint())
      return true;
  return false;
}
}  // namespace tobas
