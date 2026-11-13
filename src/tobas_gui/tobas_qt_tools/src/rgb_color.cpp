// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_qt_tools/rgb_color.hpp"

namespace tobas::qt
{
RGBColor RGBColor::mean(const RGBColor& _other) const
{
  // Convert types before computing the average to avoid overflow.
  const auto r_mean = (uint8_t)(((int)r + (int)_other.r) / 2);
  const auto g_mean = (uint8_t)(((int)g + (int)_other.g) / 2);
  const auto b_mean = (uint8_t)(((int)b + (int)_other.b) / 2);
  return RGBColor(r_mean, g_mean, b_mean);
}

bool RGBColor::operator==(const RGBColor& _rhs) const
{
  return r == _rhs.r && g == _rhs.g && b == _rhs.b;
}
}  // namespace tobas::qt
