// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <rclcpp/node.hpp>

namespace tobas::ros2
{
template <typename T>
void declareParam(
  rclcpp::Node::SharedPtr _node,
  const std::string& _name,
  const T& _default_value,
  bool _declared_ok = true)
{
  if (_node->has_parameter(_name)) {
    if (_declared_ok) {
      return;
    }
    else {
      throw rclcpp::exceptions::ParameterAlreadyDeclaredException("Parameter '" + _name + "' is already declared.");
    }
  }
  else {
    _node->declare_parameter(_name, _default_value);
  }
}
}  // namespace tobas::ros2
