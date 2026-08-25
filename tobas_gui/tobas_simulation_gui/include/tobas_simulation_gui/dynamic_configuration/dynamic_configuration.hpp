// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./suspended_load.hpp"
#include "./wind_parameters.hpp"

namespace tobas
{
namespace gui
{
namespace sim
{
class DynamicConfigWidget : public QWidget
{
  Q_OBJECT

  using self = DynamicConfigWidget;
  using super = QWidget;

public:
  explicit DynamicConfigWidget();

  void initializeRosInterfaces(rclcpp::Node::SharedPtr node, const std::string& ns);
  void clearRosInterfaces();

  bool start(std::chrono::milliseconds timeout);
  void reset();

private:
  WindParamsWidget* wind_params_;
  SuspendedLoadWidget* suspended_load_;
};
}  // namespace sim
}  // namespace gui
}  // namespace tobas
