// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <gz/sim/System.hh>
#include <gz/sim/components.hh>

namespace tobas
{
namespace gazebo
{
/* Hold all links that belong to the model. */
class ModelLinksParser
{
public:
  explicit ModelLinksParser();

  bool initialize(const gz::sim::Entity& model, const gz::sim::EntityComponentManager& ecm);

  inline const std::map<std::string, gz::sim::Entity>& getLinks() const
  {
    return links_;
  }

  inline const gz::sim::Entity& getLink(const std::string& link_name) const
  {
    return links_.at(link_name);
  }

private:
  std::map<std::string, gz::sim::Entity> links_;
};
}  // namespace gazebo
}  // namespace tobas
