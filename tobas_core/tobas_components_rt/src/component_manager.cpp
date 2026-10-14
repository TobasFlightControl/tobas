// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_components_rt/component_manager.hpp"

#include <mutex>

namespace tobas
{
namespace
{
std::mutex& componentOperationMutex()
{
  // Serialize component construction and destruction across all managers in this process.
  // Constructors can unload plugins too, e.g. when `robot_state_publisher` destroys its temporary URDF model.
  // A factory-only lock cannot prevent that unload from racing with another component's library load.
  // Runtime plugin operations initiated by node callbacks are outside this lock's scope.
  static std::mutex mutex;
  return mutex;
}
}  // namespace

void ThreadSafeComponentManager::on_load_node(
  const std::shared_ptr<rmw_request_id_t> _request_header,
  const LoadNode::Request::SharedPtr _request,
  LoadNode::Response::SharedPtr _response)
{
  const std::lock_guard lock(componentOperationMutex());
  ComponentManager::on_load_node(_request_header, _request, _response);
}

void ThreadSafeComponentManager::on_unload_node(
  const std::shared_ptr<rmw_request_id_t> _request_header,
  const UnloadNode::Request::SharedPtr _request,
  UnloadNode::Response::SharedPtr _response)
{
  const std::lock_guard lock(componentOperationMutex());
  ComponentManager::on_unload_node(_request_header, _request, _response);
}
}  // namespace tobas
