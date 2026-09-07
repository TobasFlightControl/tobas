// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <yaml-cpp/yaml.h>

namespace tobas
{
/* ct = c0 + c1 φ */
class VppMotorConstant
{
public:
  double c0;
  double c1;

  inline explicit VppMotorConstant(double _c0, double _c1) : c0(_c0), c1(_c1)
  {
  }

  inline explicit VppMotorConstant() : c0(0.0), c1(0.0)
  {
  }

  bool isValid() const;

  bool load(const YAML::Node& node);
  YAML::Node dump() const;

  inline double compute(double phi) const
  {
    return c0 + c1 * phi;
  }
};
}  // namespace tobas
