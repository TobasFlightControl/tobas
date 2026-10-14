// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_ros2_tools/node.hpp"

#include <rclcpp/node_interfaces/node_graph_interface.hpp>
#include <rclcpp/rate.hpp>

namespace ch = std::chrono;

namespace tobas::ros2
{
namespace
{
std::string makeFqn(const std::string& _name, const std::string& _ns)
{
  if (_ns.empty() || _ns == "/") {
    return "/" + _name;
  }
  return (_ns.back() == '/' ? _ns : _ns + "/") + _name;
}

bool isValidFqn(const std::string& _fqn)
{
  return _fqn.starts_with('/');
}

bool isPresent(const rclcpp::node_interfaces::NodeGraphInterface::SharedPtr& _graph, const std::string& _target_fqn)
{
  for (const auto& nn : _graph->get_node_names_and_namespaces()) {
    const auto fqn = makeFqn(nn.first, nn.second);
    if (fqn == _target_fqn) {
      return true;
    }
  }
  return false;
}
}  // namespace

bool waitUntilNodeGone(const rclcpp::Node::SharedPtr& _node, const std::string& _target_fqn, ch::milliseconds _timeout)
{
  if (!isValidFqn(_target_fqn)) {
    RCLCPP_ERROR_STREAM(_node->get_logger(), "Invalid FQN: " << _target_fqn);
    return false;
  }

  // Get the node graph.
  const auto graph = _node->get_node_graph_interface();

  // Exit immediately if the target is already absent.
  if (!isPresent(graph, _target_fqn)) {
    RCLCPP_INFO_STREAM(_node->get_logger(), "Target FQN '" << _target_fqn << "' does not exist.");
    return true;
  }

  // Check for the target node periodically.
  const auto deadline = _timeout.count() > 0 ? ch::steady_clock::now() + _timeout : ch::steady_clock::time_point::max();
  rclcpp::Rate rate(10.0, _node->get_clock());
  while (rclcpp::ok()) {
    // Handle timeout.
    if (ch::steady_clock::now() > deadline) {
      RCLCPP_WARN_STREAM(_node->get_logger(), "Timed out waiting for '" << _target_fqn << "' to shut down.");
      return false;
    }

    // Check whether the target node is still present after graph changes.
    if (!isPresent(graph, _target_fqn)) {
      RCLCPP_INFO_STREAM(_node->get_logger(), "'" << _target_fqn << "' has gone.");
      return true;
    }

    rate.sleep();
  }

  return false;
}
}  // namespace tobas::ros2
