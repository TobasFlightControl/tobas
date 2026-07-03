// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_yaml_tools/format.hpp"

#include <format>

#include <tobas_math/core.hpp>
#include <tobas_math/float.hpp>

namespace tobas
{
namespace yaml
{
std::string format(int value)
{
  return std::to_string(value);
}

/**
 * @brief Convert a floating-point value to a YAML-compatible string.
 *
 * e.g. 100 -> 100.0, 0.000012345 -> 1.2345e-05, NaN -> .nan
 */
std::string format(double value, int prec)
{
  if (std::isnan(value)) {
    return ".nan";
  }

  if (math::isInteger(value) && std::abs(static_cast<long>(value)) < math::ipow(10L, prec)) {
    // If the value has no fractional part and fewer digits than the precision,
    // it would be rounded to integer notation. Add one fractional digit.
    return std::format("{:.1f}", value);
  }
  else {
    return std::format("{:.{}g}", value, prec);
  }
}
}  // namespace yaml
}  // namespace tobas
