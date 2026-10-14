// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_gazebo_tools/utils.hpp"

#include <gz/sim/components/ChildLinkName.hh>
#include <gz/sim/components/Joint.hh>

namespace cmp = gz::sim::components;

namespace tobas
{
namespace gazebo
{
bool belongsTo(const gz::sim::Entity& _entity, const gz::sim::Entity& _target, const gz::sim::EntityComponentManager& _ecm)
{
  const auto parent = _ecm.ParentEntity(_entity);
  if (parent == gz::sim::kNullEntity) {
    return false;
  }
  else if (parent == _target) {
    return true;
  }
  else {
    return belongsTo(parent, _target, _ecm);
  }
}

gz::sim::Entity findJointWithChildLink(const gz::sim::EntityComponentManager& _ecm, const std::string& _link_name)
{
  auto res = gz::sim::kNullEntity;

  _ecm.Each<cmp::Joint, cmp::ChildLinkName>(
    [&](const gz::sim::Entity& _entity, const cmp::Joint*, const cmp::ChildLinkName* _child_link_name) -> bool
    {
      if (_child_link_name->Data() == _link_name) {
        res = _entity;
        return false;
      }
      return true;
    });

  return res;
}
}  // namespace gazebo
}  // namespace tobas
