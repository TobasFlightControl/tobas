// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tuple>

namespace tobas
{
namespace gazebo
{
std::tuple<double, double, double> boxInertia(double sx, double sy, double sz, double mass);
}  // namespace gazebo
}  // namespace tobas
