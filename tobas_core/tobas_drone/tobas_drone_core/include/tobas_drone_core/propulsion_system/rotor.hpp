// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <map>

#include "./turning_direction.hpp"

namespace tobas
{
class RotorConfig
{
public:
  using SharedPtr = std::shared_ptr<RotorConfig>;
  using ConstSharedPtr = std::shared_ptr<const RotorConfig>;

  std::string link_name = "";                          // Propeller link name.
  TurningDirection direction = TurningDirection::CCW;  // Rotation direction: CCW or CW.
  std::string tilt_joint_name = "";                    // Tilt joint name; an empty string means a fixed axis.

  virtual bool isValid() const;

  virtual bool load(const YAML::Node& node);
  virtual YAML::Node dump() const;

  /* Ratio between thrust and reaction torque [m]. */
  virtual double momentConst() const = 0;

  /* Cost of generating thrust with this actuator, used for mixing. */
  virtual double effortWeight() const = 0;

  /* CCW = 1, CW = -1 */
  inline int sign() const;
};

using RotorConfigMap = std::map<std::string, RotorConfig::SharedPtr>;  // Link Name -> RotorConfig

inline int RotorConfig::sign() const
{
  return tobas::sign(direction);
}
}  // namespace tobas
