// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_ros2_tools/node.hpp"

namespace ch = std::chrono;

namespace tobas
{
namespace ros2
{
namespace
{
std::string makeFQN(const std::string& name, const std::string& ns)
{
  if (ns.empty() || ns == "/") {
    return "/" + name;
  }
  return (ns.back() == '/' ? ns : ns + "/") + name;
}
}  // namespace

bool isPresent(const rclcpp::node_interfaces::NodeGraphInterface::SharedPtr& graph, const std::string& target_fqn)
{
  for (auto& nn : graph->get_node_names_and_namespaces()) {
    const auto fqn = makeFQN(nn.first, nn.second);
    if (fqn == target_fqn) {
      return true;
    }
  }
  return false;
}

bool waitUntilNodeGone(const rclcpp::Node::SharedPtr& node, const std::string& target_fqn, ch::nanoseconds timeout)
{
  if (target_fqn.empty() || !target_fqn.starts_with('/')) {
    RCLCPP_ERROR_STREAM(node->get_logger(), "Invalid FQN: " << target_fqn);
    return false;
  }

  // ノードグラフを取得
  const auto graph = node->get_node_graph_interface();

  // すでに居なければ即終了
  if (!isPresent(graph, target_fqn)) {
    RCLCPP_INFO_STREAM(node->get_logger(), "Target FQN \"" << target_fqn << "\" does not exist.");
    return true;
  }

  // 一定周期で目標ノードの存在を確認
  const auto deadline = ch::steady_clock::now() + timeout;
  rclcpp::Rate rate(10., node->get_clock());
  while (rclcpp::ok()) {
    // タイムアウトの処理
    if (ch::steady_clock::now() > deadline) {
      RCLCPP_WARN_STREAM(node->get_logger(), "Timed out waiting for \"" << target_fqn << "\" to shut down.");
      return false;
    }

    // グラフの変化後に目標ノードがまだ居るかを確認
    if (!isPresent(graph, target_fqn)) {
      RCLCPP_INFO_STREAM(node->get_logger(), "\"" << target_fqn << "\" has gone.");
      return true;
    }

    rate.sleep();
  }

  return false;
}
}  // namespace ros2
}  // namespace tobas
