// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_gazebo_tools/model_mass_holder.hpp"

#include <gz/sim/components/Inertial.hh>

namespace tobas::gazebo
{
ModelMassHolder::ModelMassHolder()
{
}

bool ModelMassHolder::initialize(const gz::sim::Entity& _model, const gz::sim::EntityComponentManager& _ecm)
{
  if (!model_links_parser_.initialize(_model, _ecm)) {
    gzerr << "Failed to initialize model links parser.\n";
    return false;
  }

  for (const auto& [link_name, link_entity] : model_links_parser_.getLinks()) {
    const auto inertial_entity = _ecm.Component<gz::sim::components::Inertial>(link_entity);
    if (!inertial_entity) {
      gzerr << "Failed to get the inertial entity of '" << link_name << "'.";
      return false;
    }

    const auto& inertial = inertial_entity->Data();
    mass_ += inertial.MassMatrix().Mass();
  }

  return true;
}
}  // namespace tobas::gazebo
