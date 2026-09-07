// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <random>
#include <string>
#include <string_view>

#include <rclcpp/node.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace tobas
{
namespace
{
std::string randomAlNum(size_t n)
{
  constexpr std::string_view alphabet = "0123456789abcdefghijklmnopqrstuvwxyz";
  static thread_local std::mt19937 rng(std::random_device{}());
  std::uniform_int_distribution<size_t> dist(0, alphabet.size() - 1);

  std::string s;
  for (size_t i = 0; i < n; ++i) {
    s.push_back(alphabet[dist(rng)]);
  }
  return s;
}
}  // namespace

class DummyNode : public rclcpp::Node
{
public:
  explicit DummyNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
};

DummyNode::DummyNode(const rclcpp::NodeOptions& options) : rclcpp::Node("dummy_node_" + randomAlNum(10), options)
{
}
}  // namespace tobas

RCLCPP_COMPONENTS_REGISTER_NODE(tobas::DummyNode)
