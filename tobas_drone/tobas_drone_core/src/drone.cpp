#include "tobas_drone_core/drone.hpp"

#include <tobas_yaml_tools/core.hpp>

#include "tobas_drone_core/propulsion_system/electric_propulsion_system/electric_propulsion_system.hpp"
#include "tobas_drone_core/propulsion_system/ice_propulsion_system/ice_propulsion_system.hpp"

using namespace std;
namespace fs = filesystem;

namespace tobas
{
void Drone::clear()
{
  name.clear();
  joints.clear();
  pwms.clear();
  prop.reset();
  fixed_wing.reset();
}

bool Drone::isValid() const
{
  if (name == "") {
    cerr << "This drone does not have name." << endl;
    return false;
  }

  for (const auto& [_, pwm] : pwms) {
    if (!pwm.isValid()) {
      cerr << "The configurations of PWM channel " << pwm.channel << " are invalid." << endl;
      return false;
    }
  }

  for (const auto& [_, joint] : joints) {
    if (!joint.isValid()) {
      cerr << "The configurations of joint \"" << joint.name << "\" are invalid." << endl;
      return false;
    }
  }

  if (!prop) {
    cerr << "The configurations of propulsion system is null." << endl;
    return false;
  }
  if (!prop->isValid()) {
    cerr << "The configurations of propulsion system are invalid." << endl;
    return false;
  }

  if (fixed_wing && !fixed_wing->isValid()) {
    cerr << "The configurations of fixed wing are invalid." << endl;
    return false;
  }

  if (num_sbus_channels < tobas::kMinSbusChannels || tobas::kMaxSbusChannels < num_sbus_channels) {
    cerr << "The number of sbus channels is invalid." << endl;
    return false;
  }

  return true;
}

bool Drone::load(const YAML::Node& root_node)
{
  clear();

  // Name
  if (!yaml::load(kNameKey, root_node, name)) {
    return false;
  }

  // Joints
  const auto joints_node = root_node[kJointsKey];
  if (!joints_node.IsDefined()) {
    cerr << "\"" << kJointsKey << "\" is not defined." << endl;
    return false;
  }
  if (!joints_node.IsSequence()) {
    cerr << "\"" << kJointsKey << "\" must be a sequence." << endl;
    return false;
  }
  for (const auto& joint_node : joints_node) {
    JointConfig joint;
    if (!joint.load(joint_node)) {
      cerr << "Failed to load the configurations of joints." << endl;
      return false;
    }
    joints[joint.name] = joint;
  }

  // PWM
  const auto pwms_node = root_node[kPwmsKey];
  if (!pwms_node.IsDefined()) {
    cerr << "\"" << kPwmsKey << "\" is not defined." << endl;
    return false;
  }
  if (!pwms_node.IsSequence()) {
    cerr << "\"" << kPwmsKey << "\" must be a sequence." << endl;
    return false;
  }
  for (const auto& pwm_node : pwms_node) {
    PwmConfig pwm;
    if (!pwm.load(pwm_node)) {
      cerr << "Failed to load the configurations of PWM." << endl;
      return false;
    }
    pwms[pwm.name] = pwm;
  }

  // Propulsion System
  PropulsionSystem prop_type;
  if (!yaml::load(kPropulsionSystemTypeKey, root_node, prop_type)) {
    return false;
  }

  const auto prop_node = root_node[kPropulsionSystemKey];
  if (!prop_node.IsDefined()) {
    cerr << "\"" << kPropulsionSystemKey << "\" is not defined." << endl;
    return false;
  }

  switch (prop_type) {
    case PropulsionSystem::kElectric: {
      const auto eprop = make_shared<ElectricPropulsionSystemConfig>();
      if (!eprop->load(prop_node)) {
        cerr << "Failed to load the configurations of electric propulsion system." << endl;
        return false;
      }
      prop = static_pointer_cast<PropulsionSystemConfig>(eprop);
      break;
    }
    case PropulsionSystem::kIce: {
      const auto iprop = make_shared<ICEPropulsionSystemConfig>();
      if (!iprop->load(prop_node)) {
        cerr << "Failed to load the configurations of ICE propulsion system." << endl;
        return false;
      }
      prop = static_pointer_cast<PropulsionSystemConfig>(iprop);
      break;
    }
    default: {
      cerr << "Invalid propulsion system type: " << (int)prop_type << endl;
      return false;
    }
  }

  // Fixed Wing
  const auto fw_node = root_node[kFixedWingKey];
  if (fw_node.IsDefined()) {
    fixed_wing = make_shared<FixedWingConfig>();
    if (!fixed_wing->load(fw_node)) {
      cerr << "Failed to load the configurations of fixed wing." << endl;
      return false;
    }
  }
  else {
    cout << "\"" << kFixedWingKey << "\" is not defined." << endl;
    fixed_wing.reset();
  }

  // S.BUS Channels
  if (!yaml::load(kNumSbusChannelsKey, root_node, num_sbus_channels)) {
    return false;
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
  for (const auto& [_, joint] : joints) {
    node[kJointsKey].push_back(joint.dump());
  }

  // PWM
  node[kPwmsKey] = YAML::Node(YAML::NodeType::Sequence);
  for (const auto& [_, pwm] : pwms) {
    node[kPwmsKey].push_back(pwm.dump());
  }

  // Propulsion System
  node[kPropulsionSystemTypeKey] = prop->type();
  node[kPropulsionSystemKey] = prop->dump();

  // Fixed Wing
  if (fixed_wing) {
    node[kFixedWingKey] = fixed_wing->dump();
  }

  // S.BUS Channels
  node[kNumSbusChannelsKey] = num_sbus_channels;

  return node;
}

bool Drone::load(const fs::path& path)
{
  if (path.extension() != kDroneExt) {
    cerr << "Invalid extension: " << path.extension() << endl;
    return false;
  }

  YAML::Node node;
  if (!yaml::load(path, node)) {
    return false;
  }

  if (!load(node)) {
    cerr << "Failed to load drone." << endl;
    return false;
  }

  return true;
}

bool Drone::save(const fs::path& path) const
{
  if (path.extension() != kDroneExt) {
    cerr << "Invalid extension: " << path.extension() << endl;
    return false;
  }

  const auto node = dump();

  if (!yaml::save(path, node)) {
    return false;
  }

  return true;
}

bool Drone::hasServoJoint() const
{
  for (const auto& [_, joint] : joints) {
    if (joint.isServoJoint()) {
      return true;
    }
  }
  return false;
}
}  // namespace tobas
