#include "tobas_gazebo_system_plugins/world.hpp"

#include <gz/sim/World.hh>
#include <gz/sim/components/World.hh>

namespace cmp = gz::sim::components;

namespace gazebo
{
std::expected<gz::math::SphericalCoordinates, const char*>
getWorldSphericalCoordinates(const gz::sim::EntityComponentManager& ecm)
{
  const auto world_entity = ecm.EntityByComponents(cmp::World());
  if (world_entity == gz::sim::kNullEntity) {
    return std::unexpected("World entity not found.");
  }

  const gz::sim::World world(world_entity);
  if (!world.Valid(ecm)) {
    return std::unexpected("Failed to get the world model.");
  }

  const auto sc_opt = world.SphericalCoordinates(ecm);
  if (!sc_opt) {
    return std::unexpected("No spherical coordinates on the world.");
  }

  return sc_opt.value();
}
}  // namespace gazebo
