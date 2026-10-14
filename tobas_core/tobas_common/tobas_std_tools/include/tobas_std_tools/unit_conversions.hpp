// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <cmath>

namespace tobas::st
{
/** degree -> radian */
inline constexpr double deg2rad(double _deg)
{
  return _deg * (M_PI / 180.0);
}

/** radian -> degree */
inline constexpr double rad2deg(double _rad)
{
  return _rad * (180.0 / M_PI);
}

/** rpm -> rad/s */
inline constexpr double rpm2rps(double _rpm)
{
  return _rpm * (M_PI / 30.0);
}

/** rad/s -> rpm */
inline constexpr double rps2rpm(double _rps)
{
  return _rps * (30.0 / M_PI);
}

/** inch -> m */
inline constexpr double inch2meter(double _inch)
{
  return _inch * 0.0254;
}

/** m -> inch */
inline constexpr double meter2inch(double _meter)
{
  return _meter / 0.0254;
}

/** feet -> m */
inline constexpr double feet2meter(double _feet)
{
  return _feet * 0.3048;
}

/** m -> feet */
inline constexpr double meter2feet(double _meter)
{
  return _meter / 0.3048;
}

/** yard -> m */
inline constexpr double yard2meter(double _yard)
{
  return _yard * 0.9144;
}

/** m -> yard */
inline constexpr double meter2yard(double _meter)
{
  return _meter / 0.9144;
}
}  // namespace tobas::st
