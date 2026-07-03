// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <cmath>

namespace tobas
{
namespace st
{
static constexpr double kGravity = 9.80665;           // [m/s^2] Gravitational acceleration.
static constexpr double kStandardAirDensity = 1.225;  // [kg/m^3] Air density in the standard atmosphere.
static constexpr double kGeomagScale = 0.5;           // [G] Magnitude of the geomagnetic flux density.

static constexpr double kDeg2Rad = M_PI / 180;  // deg -> rad
static constexpr double kRad2Deg = 180 / M_PI;  // rad -> deg

static constexpr double kRpmToRps = M_PI / 30.;  // RPM -> rad/s
static constexpr double kRpsToRpm = 30. / M_PI;  // rad/s -> RPM

static constexpr double kFeetToMeter = 0.3048;
static constexpr double kMeterToFeet = 1 / kFeetToMeter;
}  // namespace st
}  // namespace tobas
