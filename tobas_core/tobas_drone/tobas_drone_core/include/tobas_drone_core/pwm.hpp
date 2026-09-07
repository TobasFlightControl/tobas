// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <map>
#include <string>

#include <yaml-cpp/yaml.h>

#include <tobas_math/core.hpp>

namespace tobas
{
class PwmConfig;
using PwmConfigMap = std::map<std::string, PwmConfig>;  // Name -> PwmConfig

class PwmConfig
{
public:
  uint32_t channel = 0;
  std::string name = "";
  std::pair<double, double> period_range = { 1000, 2000 };  // [us]
  std::pair<double, double> value_range = { 0, 0 };         // Range of values corresponding to PWM.

  bool isValid() const;

  bool load(const YAML::Node& node);
  YAML::Node dump() const;

  inline double periodFromValue(double value) const;

private:
  inline double clampPeriod(double period) const;
};

inline double PwmConfig::periodFromValue(double value) const
{
  const auto period =
    math::remap(value, value_range.first, value_range.second, period_range.first, period_range.second);
  return clampPeriod(period);
}

inline double PwmConfig::clampPeriod(double period) const
{
  if (period_range.first < period_range.second) {
    return std::clamp(period, period_range.first, period_range.second);
  }
  else {
    return std::clamp(period, period_range.second, period_range.first);
  }
}
}  // namespace tobas
