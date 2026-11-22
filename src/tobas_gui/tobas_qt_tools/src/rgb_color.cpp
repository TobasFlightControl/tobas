// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_qt_tools/rgb_color.hpp"

#include <cstdint>

namespace tobas::qt
{
RGBColor RGBColor::mean(const RGBColor& _other) const
{
  // Convert types before computing the average to avoid overflow.
  const auto r_mean = static_cast<uint8_t>((static_cast<int>(r) + static_cast<int>(_other.r)) / 2);
  const auto g_mean = static_cast<uint8_t>((static_cast<int>(g) + static_cast<int>(_other.g)) / 2);
  const auto b_mean = static_cast<uint8_t>((static_cast<int>(b) + static_cast<int>(_other.b)) / 2);
  return RGBColor(r_mean, g_mean, b_mean);
}

bool RGBColor::operator==(const RGBColor& _rhs) const
{
  return r == _rhs.r && g == _rhs.g && b == _rhs.b;
}
}  // namespace tobas::qt
