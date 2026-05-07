// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <expected>

#include <gz/math/SphericalCoordinates.hh>
#include <gz/sim/EntityComponentManager.hh>

namespace tobas
{
namespace gazebo
{
std::expected<std::string, const char*> getWorldName(const gz::sim::EntityComponentManager& ecm);

std::expected<gz::math::SphericalCoordinates, const char*>
getWorldSphericalCoordinates(const gz::sim::EntityComponentManager& ecm);
}  // namespace gazebo
}  // namespace tobas
