#include <tobas_yaml_tools/core.hpp>

#include "tobas_drone_core/drone.hpp"
#include "tobas_drone_core/propulsion_system/electric_propulsion_system/electric_propulsion_system.hpp"
#include "tobas_drone_core/propulsion_system/ice_propulsion_system/ice_propulsion_system.hpp"

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

  for (const auto& [_, pwm] : pwms)
  {
    if (!pwm.isValid())
    {
      cerr << "The configurations of PWM channel " << pwm.channel << " are invalid." << endl;
      return false;
    }
  }

  for (const auto& [_, joint] : joints)
  {
    if (!joint.isValid())
    {
      cerr << "The configurations of joint \"" << joint.name << "\" are invalid." << endl;
      return false;
    }
  }

  if (!prop)
  {
    cerr << "The configurations of propulsion system is null." << endl;
    return false;
  }
  if (!prop->isValid())
  {
    cerr << "The configurations of propulsion system are invalid." << endl;
    return false;
  }

  if (fixed_wing && !fixed_wing->isValid())
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

  // PWM
  pwms.clear();
  if (!node[kPwmsKey].IsSequence())
  {
    cerr << "PWM field is not defined." << endl;
    return false;
  }
  for (const auto& pwm_node : node[kPwmsKey])
  {
    PwmConfig pwm;
    if (!pwm.load(pwm_node))
    {
      cerr << "Failed to load the configurations of PWM." << endl;
      return false;
    }
    pwms[pwm.name] = pwm;
  }

  // Propulsion system
  propulsion_system_t prop_type;
  if (!yaml::load(kPropulsionSystemTypeKey, node, prop_type))
    return false;

  if (!node[kPropulsionSystemKey].IsDefined())
  {
    cerr << "Propulsion system field is not defined." << endl;
    return false;
  }
  const auto prop_node = node[kPropulsionSystemKey];

  switch (prop_type)
  {
    case propulsion_system_t::ELECTRIC:
    {
      const auto eprop = make_shared<ElectricPropulsionSystemConfig>();
      if (!eprop->load(prop_node))
      {
        cerr << "Failed to load the configurations of electric propulsion system." << endl;
        return false;
      }
      prop = static_pointer_cast<PropulsionSystemConfig>(eprop);
      break;
    }
    case propulsion_system_t::ICE:
    {
      const auto iprop = make_shared<ICEPropulsionSystemConfig>();
      if (!iprop->load(prop_node))
      {
        cerr << "Failed to load the configurations of ICE propulsion system." << endl;
        return false;
      }
      prop = static_pointer_cast<PropulsionSystemConfig>(iprop);
      break;
    }
    default:
    {
      cerr << "Invalid propulsion system type: " << (int)prop_type << endl;
      return false;
    }
  }

  // Fixed wing
  if (node[kFixedWingKey].IsDefined())
  {
    fixed_wing = make_shared<FixedWingConfig>();
    if (!fixed_wing->load(node[kFixedWingKey]))
    {
      cerr << "Failed to load the configurations of fixed wing." << endl;
      return false;
    }
  }
  else
  {
    cout << "\"" << kFixedWingKey << "\" is not defined." << endl;
    fixed_wing.reset();
  }

  return true;
}

YAML::Node Drone::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  // Name
  node[kNameKey] = name;

  // Joints
  node[kJointsKey] = YAML::Node(YAML::NodeType::Sequence);
  for (const auto& [_, joint] : joints)
    node[kJointsKey].push_back(joint.dump());

  // PWM
  node[kPwmsKey] = YAML::Node(YAML::NodeType::Sequence);
  for (const auto& [_, pwm] : pwms)
    node[kPwmsKey].push_back(pwm.dump());

  // Propulsion system
  node[kPropulsionSystemTypeKey] = prop->type();
  node[kPropulsionSystemKey] = prop->dump();

  // Fixed wing
  if (fixed_wing)
    node[kFixedWingKey] = fixed_wing->dump();

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
