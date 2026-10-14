// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <map>
#include <string>

#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>

namespace tobas::gazebo
{
/** Hold all links that belong to the model. */
class ModelLinksParser
{
public:
  explicit ModelLinksParser();

  bool initialize(const gz::sim::Entity& _model, const gz::sim::EntityComponentManager& _ecm);

  inline const std::map<std::string, gz::sim::Entity>& getLinks() const
  {
    return links_;
  }

  inline const gz::sim::Entity& getLink(const std::string& _link_name) const
  {
    return links_.at(_link_name);
  }

private:
  std::map<std::string, gz::sim::Entity> links_;
};
}  // namespace tobas::gazebo
