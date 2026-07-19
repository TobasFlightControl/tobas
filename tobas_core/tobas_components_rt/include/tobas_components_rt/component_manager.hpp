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

protected:
  virtual void on_load_node(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<LoadNode::Request> request,
    std::shared_ptr<LoadNode::Response> response) override;

  virtual void on_unload_node(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<UnloadNode::Request> request,
    std::shared_ptr<UnloadNode::Response> response) override;
};
}  // namespace tobas
