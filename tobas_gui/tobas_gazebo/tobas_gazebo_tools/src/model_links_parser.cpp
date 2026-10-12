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

bool ModelLinksParser::initialize(const gz::sim::Entity& _model, const gz::sim::EntityComponentManager& _ecm)
{
  if (!_ecm.Component<cmp::Model>(_model)) {
    ignerr << "Model does not exist." << std::endl;
    return false;
  }

  links_.clear();

  _ecm.Each<cmp::Link, cmp::Name>(
    [&](const gz::sim::Entity& _entity, const cmp::Link*, const cmp::Name* _name) -> bool
    {
      if (belongsTo(_entity, _model, _ecm)) {
        links_[_name->Data()] = _entity;
      }
      return true;
    });

  return true;
}
}  // namespace gazebo
}  // namespace tobas
