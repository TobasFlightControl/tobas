// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <filesystem>

#include <yaml-cpp/yaml.h>

#include "./fixed_wing/fixed_wing.hpp"
#include "./joint/joint.hpp"
#include "./propulsion_system/propulsion_system.hpp"
#include "./propulsion_system/type.hpp"
#include "./pwm.hpp"

namespace tobas
{
/**
 * @brief Class that holds only the information required to describe a drone.
 */
class Drone
{
  static constexpr char kNameKey[] = "name";
  static constexpr char kJointsKey[] = "joints";
  static constexpr char kPwmsKey[] = "pwms";
  static constexpr char kPropulsionSystemTypeKey[] = "propulsion_system_type";
  static constexpr char kPropulsionSystemKey[] = "propulsion_system";
  static constexpr char kFixedWingKey[] = "fixed_wing";
  static constexpr char kNumSbusChannelsKey[] = "num_sbus_channels";

public:
  using SharedPtr = std::shared_ptr<Drone>;
  using ConstSharedPtr = std::shared_ptr<const Drone>;

  std::string name = "";                   // The name of this drone
  JointConfigMap joints;                   // The servo joint configuration (joint name -> config)
  PwmConfigMap pwms;                       // The PWM configuration (joint name -> config)
  PropulsionSystemConfig::SharedPtr prop;  // The propulsion system configuration
  FixedWingConfig::SharedPtr fixed_wing;   // The fixed wing configuration
  uint32_t num_sbus_channels = 0;          // The number of S.BUS channels

  inline bool empty() const;

  void clear();

  bool isValid() const;

  bool load(const YAML::Node& node);
  YAML::Node dump() const;

  bool load(const std::filesystem::path& path);
  bool save(const std::filesystem::path& path) const;

  bool hasServoJoint() const;
};

inline bool Drone::empty() const
{
  return prop == nullptr;
}
}  // namespace tobas
