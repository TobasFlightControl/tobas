// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_gazebo_system_plugins/world.hpp"

#include <gz/sim/World.hh>
#include <gz/sim/components/World.hh>

namespace cmp = gz::sim::components;

namespace tobas
{
namespace gazebo
{
std::expected<gz::sim::World, const char*> getWorld(const gz::sim::EntityComponentManager& ecm)
{
  const auto world_entity = ecm.EntityByComponents(cmp::World());
  if (world_entity == gz::sim::kNullEntity) {
    return std::unexpected("World entity not found.");
  }

  const gz::sim::World world(world_entity);
  if (!world.Valid(ecm)) {
    return std::unexpected("Failed to get the world model.");
  }

  return world;
}

std::expected<std::string, const char*> getWorldName(const gz::sim::EntityComponentManager& ecm)
{
  const auto world = getWorld(ecm);
  if (!world) {
    return std::unexpected(world.error());
  }

  const auto name = world->Name(ecm);
  if (!name) {
    return std::unexpected("No world name.");
  }

  return *name;
}

std::expected<gz::math::SphericalCoordinates, const char*>
getWorldSphericalCoordinates(const gz::sim::EntityComponentManager& ecm)
{
  const auto world = getWorld(ecm);
  if (!world) {
    return std::unexpected(world.error());
  }

  const auto sc = world->SphericalCoordinates(ecm);
  if (!sc) {
    return std::unexpected("No spherical coordinates on the world.");
  }

  return *sc;
}
}  // namespace gazebo
}  // namespace tobas
