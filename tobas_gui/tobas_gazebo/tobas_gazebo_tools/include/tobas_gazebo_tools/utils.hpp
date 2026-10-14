// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <string>

#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>

namespace tobas::gazebo
{
/** Get the component directly under an entity. Create a new one if it does not exist. */
template <typename CompType>
CompType* getComponent(const gz::sim::Entity& _entity, gz::sim::EntityComponentManager& _ecm)
{
  if (_ecm.EntityHasComponentType(_entity, CompType().TypeId())) {
    return _ecm.Component<CompType>(_entity);
  }
  else {
    return _ecm.CreateComponent(_entity, CompType());
  }
}

bool belongsTo(
  const gz::sim::Entity& _entity,
  const gz::sim::Entity& _target,
  const gz::sim::EntityComponentManager& _ecm);

gz::sim::Entity findJointWithChildLink(const gz::sim::EntityComponentManager& _ecm, const std::string& _link_name);
}  // namespace tobas::gazebo
