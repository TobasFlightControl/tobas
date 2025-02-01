#pragma once

#include <gz/sim/System.hh>
#include <gz/sim/components.hh>

namespace gazebo
{
bool belongsTo(
  const gz::sim::Entity& entity,
  const gz::sim::Entity& target,
  const gz::sim::EntityComponentManager& ecm);

std::optional<gz::sim::Entity>
findJointWithChildLink(const gz::sim::EntityComponentManager& ecm, const std::string& link_name);
}  // namespace gazebo
