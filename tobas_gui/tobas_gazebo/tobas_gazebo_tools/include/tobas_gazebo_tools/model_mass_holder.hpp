// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./model_links_parser.hpp"

namespace tobas
{
namespace gazebo
{
/** Hold the total mass of the model. */
class ModelMassHolder
{
public:
  explicit ModelMassHolder();

  bool initialize(const gz::sim::Entity& _model, const gz::sim::EntityComponentManager& _ecm);

  inline const double& getMass() const
  {
    return mass_;
  }

private:
  double mass_;
  ModelLinksParser model_links_parser_;
};
}  // namespace gazebo
}  // namespace tobas
