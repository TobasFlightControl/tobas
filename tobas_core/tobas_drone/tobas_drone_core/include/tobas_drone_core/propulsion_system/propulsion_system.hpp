// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <yaml-cpp/yaml.h>

#include "./rotor.hpp"
#include "./type.hpp"

namespace tobas
{
class PropulsionSystemConfig
{
protected:
  static constexpr char kRotorsKey[] = "rotors";

public:
  using SharedPtr = std::shared_ptr<PropulsionSystemConfig>;
  using ConstSharedPtr = std::shared_ptr<const PropulsionSystemConfig>;

  RotorConfigMap rotors;

  virtual void clear();

  virtual bool isValid() const = 0;

  virtual bool load(const YAML::Node& node) = 0;
  virtual YAML::Node dump() const = 0;

  virtual PropulsionSystem type() const = 0;

  virtual double minSpeed(const std::string& link_name) = 0;
  virtual double maxSpeed(const std::string& link_name) = 0;

  virtual double minThrust(const std::string& link_name) = 0;
  virtual double maxThrust(const std::string& link_name) = 0;

  virtual double thrustFromThrottle(const std::string& link_name, double throttle) = 0;

  inline size_t numRotors() const;
};

inline size_t PropulsionSystemConfig::numRotors() const
{
  return rotors.size();
}
}  // namespace tobas
