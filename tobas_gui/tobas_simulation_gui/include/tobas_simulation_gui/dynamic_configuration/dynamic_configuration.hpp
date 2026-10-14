// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./fixed_load.hpp"
#include "./suspended_load.hpp"
#include "./wind_parameters.hpp"

namespace tobas::gui::sim
{
class DynamicConfigWidget : public QWidget
{
  Q_OBJECT

public:
  explicit DynamicConfigWidget();

  void reset();
  void initializeRosInterfaces(rclcpp::Node::SharedPtr _node, const std::string& _ns);
  void clearRosInterfaces();

private:
  WindParamsWidget* wind_params_;
  FixedLoadWidget* fixed_load_;
  SuspendedLoadWidget* suspended_load_;
};
}  // namespace tobas::gui::sim
