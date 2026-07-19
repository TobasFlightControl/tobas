// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <rclcpp_components/component_manager.hpp>

namespace tobas
{
class ThreadSafeComponentManager : public rclcpp_components::ComponentManager
{
public:
  using ComponentManager::ComponentManager;

  virtual std::shared_ptr<rclcpp_components::NodeFactory>
  create_component_factory(const ComponentResource& resource) override;
};
}  // namespace tobas
