// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_gazebo_tools/model_links_parser.hpp"

#include <gz/common/Console.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>

#include "tobas_gazebo_tools/utils.hpp"

namespace cmp = gz::sim::components;

namespace tobas
{
namespace gazebo
{
ModelLinksParser::ModelLinksParser()
{
}

bool ModelLinksParser::initialize(const gz::sim::Entity& model, const gz::sim::EntityComponentManager& ecm)
{
  if (!ecm.Component<cmp::Model>(model)) {
    ignerr << "Model does not exist." << std::endl;
    return false;
  }

  links_.clear();

  ecm.Each<cmp::Link, cmp::Name>(
    [&](const gz::sim::Entity& entity, const cmp::Link*, const cmp::Name* name) -> bool
    {
      if (belongsTo(entity, model, ecm)) {
        links_[name->Data()] = entity;
      }
      return true;
    });

  return true;
}
}  // namespace gazebo
}  // namespace tobas
