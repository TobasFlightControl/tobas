// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_drone_core/drone.hpp"

#include <tobas_constants/rc_input.hpp>
#include <tobas_yaml_tools/core.hpp>

#include "tobas_drone_core/propulsion_system/electric_propulsion_system/electric_propulsion_system.hpp"
#include "tobas_drone_core/propulsion_system/ice_propulsion_system/ice_propulsion_system.hpp"

namespace fs = std::filesystem;

namespace tobas
{
namespace
{
constexpr char kNameKey[] = "name";
constexpr char kJointsKey[] = "joints";
constexpr char kPwmsKey[] = "pwms";
constexpr char kPropulsionSystemTypeKey[] = "propulsion_system_type";
constexpr char kPropulsionSystemKey[] = "propulsion_system";
constexpr char kFixedWingKey[] = "fixed_wing";
constexpr char kNumSbusChannelsKey[] = "num_sbus_channels";
}  // namespace

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
    std::cerr << "This drone does not have name." << std::endl;
    return false;
  }

  for (const auto& [_, pwm] : pwms) {
    if (!pwm.isValid()) {
      std::cerr << "The configuration of PWM channel " << pwm.channel << " is invalid." << std::endl;
      return false;
    }
  }

  for (const auto& [_, joint] : joints) {
    if (!joint.isValid()) {
      std::cerr << "The configuration of joint '" << joint.name << "' is invalid." << std::endl;
      return false;
    }
  }

  if (!prop) {
    std::cerr << "The configuration of propulsion system is null." << std::endl;
    return false;
  }
  if (!prop->isValid()) {
    std::cerr << "The configuration of propulsion system is invalid." << std::endl;
    return false;
  }

  if (fixed_wing && !fixed_wing->isValid()) {
    std::cerr << "The configuration of fixed wing is invalid." << std::endl;
    return false;
  }

  if (num_sbus_channels < kMinSbusChannels || kMaxSbusChannels < num_sbus_channels) {
    std::cerr << "The number of sbus channels is invalid." << std::endl;
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
    std::cerr << "'" << kJointsKey << "' is not defined." << std::endl;
    return false;
  }
  if (!joints_node.IsSequence()) {
    std::cerr << "'" << kJointsKey << "' must be a sequence." << std::endl;
    return false;
  }
  for (const auto& joint_node : joints_node) {
    JointConfig joint;
    if (!joint.load(joint_node)) {
      std::cerr << "Failed to load the configuration of joints." << std::endl;
      return false;
    }
    joints[joint.name] = joint;
  }

  // PWM
  const auto pwms_node = root_node[kPwmsKey];
  if (!pwms_node.IsDefined()) {
    std::cerr << "'" << kPwmsKey << "' is not defined." << std::endl;
    return false;
  }
  if (!pwms_node.IsSequence()) {
    std::cerr << "'" << kPwmsKey << "' must be a sequence." << std::endl;
    return false;
  }
  for (const auto& pwm_node : pwms_node) {
    PwmConfig pwm;
    if (!pwm.load(pwm_node)) {
      std::cerr << "Failed to load the configuration of PWM." << std::endl;
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
    std::cerr << "'" << kPropulsionSystemKey << "' is not defined." << std::endl;
    return false;
  }

  switch (prop_type) {
    case PropulsionSystem::kElectric: {
      const auto eprop = std::make_shared<ElectricPropulsionSystemConfig>();
      if (!eprop->load(prop_node)) {
        std::cerr << "Failed to load the configuration of electric propulsion system." << std::endl;
        return false;
      }
      prop = std::static_pointer_cast<PropulsionSystemConfig>(eprop);
      break;
    }
    case PropulsionSystem::kIce: {
      const auto iprop = std::make_shared<IcePropulsionSystemConfig>();
      if (!iprop->load(prop_node)) {
        std::cerr << "Failed to load the configuration of ICE propulsion system." << std::endl;
        return false;
      }
      prop = std::static_pointer_cast<PropulsionSystemConfig>(iprop);
      break;
    }
    default: {
      std::cerr << "Invalid propulsion system type: " << (int)prop_type << std::endl;
      return false;
    }
  }

  // Fixed Wing
  const auto fw_node = root_node[kFixedWingKey];
  if (fw_node.IsDefined()) {
    fixed_wing = std::make_shared<FixedWingConfig>();
    if (!fixed_wing->load(fw_node)) {
      std::cerr << "Failed to load the configuration of fixed wing." << std::endl;
      return false;
    }
  }
  else {
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
  const auto node = yaml::load(path);
  if (!node) {
    std::cerr << node.error() << std::endl;
    return false;
  }

  if (!load(*node)) {
    std::cerr << "Failed to load drone." << std::endl;
    return false;
  }

  return true;
}

bool Drone::save(const fs::path& path) const
{
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
