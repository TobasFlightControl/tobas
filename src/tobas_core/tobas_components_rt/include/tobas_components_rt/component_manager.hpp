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

protected:
  void on_load_node(
    const std::shared_ptr<rmw_request_id_t> _request_header,
    const LoadNode::Request::SharedPtr _request,
    LoadNode::Response::SharedPtr _response) override;

  void on_unload_node(
    const std::shared_ptr<rmw_request_id_t> _request_header,
    const UnloadNode::Request::SharedPtr _request,
    UnloadNode::Response::SharedPtr _response) override;
};
}  // namespace tobas
