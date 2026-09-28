// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_drone_core/pwm.hpp"

#include <tobas_yaml_tools/convert/pair.hpp>
#include <tobas_yaml_tools/core.hpp>

namespace tobas
{
namespace
{
constexpr char kChannelKey[] = "channel";
constexpr char kNameKey[] = "name";
constexpr char kPeriodRangeKey[] = "period_range";
constexpr char kValueRangeKey[] = "value_range";
}  // namespace

std::expected<void, std::string> PwmConfig::validate() const
{
  if (name.empty()) {
    return std::unexpected("PWM name is empty.");
  }

  if (period_range.first <= 0.0 || period_range.second <= 0.0) {
    return std::unexpected("PWM period range of '" + name + "' must be positive.");
  }

  return {};
}

bool PwmConfig::load(const YAML::Node& node)
{
  if (!yaml::load(kChannelKey, node, channel)) {
    return false;
  }

  if (!yaml::load(kNameKey, node, name)) {
    return false;
  }

  if (!yaml::load(kPeriodRangeKey, node, period_range)) {
    return false;
  }

  if (!yaml::load(kValueRangeKey, node, value_range)) {
    return false;
  }

  return true;
}

YAML::Node PwmConfig::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[kChannelKey] = channel;
  node[kNameKey] = name;
  node[kPeriodRangeKey] = period_range;
  node[kValueRangeKey] = value_range;

  return node;
}
}  // namespace tobas
