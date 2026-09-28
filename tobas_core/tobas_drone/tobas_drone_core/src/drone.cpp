// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_drone_core/drone.hpp"

#include <ranges>

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

std::expected<void, std::string> Drone::validate() const
{
  if (name.empty()) {
    return std::unexpected("Drone name is empty.");
  }

  for (const auto& [_, pwm] : pwms) {
    if (const auto result = pwm.validate(); !result) {
      return std::unexpected("PWM channel " + std::to_string(pwm.channel) + ": " + result.error());
    }
  }

  for (const auto& [_, joint] : joints) {
    if (const auto result = joint.validate(); !result) {
      return std::unexpected("Joint '" + joint.name + "': " + result.error());
    }
  }

  if (!prop) {
    return std::unexpected("Propulsion system configuration is missing.");
  }
  if (const auto result = prop->validate(); !result) {
    return std::unexpected("Propulsion system: " + result.error());
  }

  if (fixed_wing) {
    if (const auto result = fixed_wing->validate(); !result) {
      return std::unexpected("Fixed wing: " + result.error());
    }
  }

  if (num_sbus_channels < kMinSbusChannels || kMaxSbusChannels < num_sbus_channels) {
    return std::unexpected(
      "The number of S.BUS channels must be between " + std::to_string(kMinSbusChannels) + " and " +
      std::to_string(kMaxSbusChannels) + ".");
  }

  return {};
}

std::expected<void, std::string> Drone::load(const YAML::Node& root_node)
{
  if (!root_node.IsDefined() || !root_node.IsMap()) {
    return std::unexpected("Drone node must be a map.");
  }

  clear();

  // Name
  if (const auto result = yaml::load(kNameKey, root_node, name); !result) {
    return result;
  }

  // Joints
  const auto joints_node = root_node[kJointsKey];
  if (!joints_node.IsDefined()) {
    return std::unexpected(std::string("'") + kJointsKey + "' is not defined.");
  }
  if (!joints_node.IsSequence()) {
    return std::unexpected(std::string("'") + kJointsKey + "' must be a sequence.");
  }
  for (const auto [idx, joint_node] : std::views::enumerate(joints_node)) {
    JointConfig joint;
    if (const auto result = joint.load(joint_node); !result) {
      return std::unexpected("Joints[" + std::to_string(idx) + "]: " + result.error());
    }
    joints[joint.name] = joint;
  }

  // PWM
  const auto pwms_node = root_node[kPwmsKey];
  if (!pwms_node.IsDefined()) {
    return std::unexpected(std::string("'") + kPwmsKey + "' is not defined.");
  }
  if (!pwms_node.IsSequence()) {
    return std::unexpected(std::string("'") + kPwmsKey + "' must be a sequence.");
  }
  for (const auto [idx, pwm_node] : std::views::enumerate(pwms_node)) {
    PwmConfig pwm;
    if (const auto result = pwm.load(pwm_node); !result) {
      return std::unexpected("PWM[" + std::to_string(idx) + "]: " + result.error());
    }
    pwms[pwm.name] = pwm;
  }

  // Propulsion System
  PropulsionSystem prop_type;
  if (const auto result = yaml::load(kPropulsionSystemTypeKey, root_node, prop_type); !result) {
    return result;
  }

  const auto prop_node = root_node[kPropulsionSystemKey];
  if (!prop_node.IsDefined()) {
    return std::unexpected(std::string("'") + kPropulsionSystemKey + "' is not defined.");
  }

  switch (prop_type) {
    case PropulsionSystem::kElectric: {
      const auto eprop = std::make_shared<ElectricPropulsionSystemConfig>();
      if (const auto result = eprop->load(prop_node); !result) {
        return std::unexpected("Electric propulsion system: " + result.error());
      }
      prop = std::static_pointer_cast<PropulsionSystemConfig>(eprop);
      break;
    }
    case PropulsionSystem::kIce: {
      const auto iprop = std::make_shared<IcePropulsionSystemConfig>();
      if (const auto result = iprop->load(prop_node); !result) {
        return std::unexpected("ICE propulsion system: " + result.error());
      }
      prop = std::static_pointer_cast<PropulsionSystemConfig>(iprop);
      break;
    }
    default: {
      return std::unexpected("Invalid propulsion system type: " + std::to_string(static_cast<int>(prop_type)));
    }
  }

  // Fixed Wing
  const auto fw_node = root_node[kFixedWingKey];
  if (fw_node.IsDefined()) {
    fixed_wing = std::make_shared<FixedWingConfig>();
    if (const auto result = fixed_wing->load(fw_node); !result) {
      return std::unexpected("Fixed wing: " + result.error());
    }
  }
  else {
    fixed_wing.reset();
  }

  // S.BUS Channels
  if (const auto result = yaml::load(kNumSbusChannelsKey, root_node, num_sbus_channels); !result) {
    return result;
  }

  return {};
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

std::expected<void, std::string> Drone::load(const fs::path& path)
{
  const auto node = yaml::load(path);
  if (!node) {
    return std::unexpected(node.error());
  }

  if (const auto result = load(*node); !result) {
    return std::unexpected("Failed to load drone from '" + path.string() + "': " + result.error());
  }

  return {};
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
