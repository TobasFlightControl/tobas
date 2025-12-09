#pragma once

#include <expected>

#include <gz/math/SphericalCoordinates.hh>
#include <gz/sim/EntityComponentManager.hh>

namespace gazebo
{
std::expected<gz::math::SphericalCoordinates, const char*>
getWorldSphericalCoordinates(const gz::sim::EntityComponentManager& ecm);
}  // namespace gazebo
