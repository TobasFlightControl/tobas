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
  explicit DynamicConfigWidget(rclcpp::Node::SharedPtr node);

  void updateNamespace(const std::string& ns);

  bool start();
  void reset();

private:
  WindParamsWidget* wind_params_;
  SuspendedLoadWidget* suspended_load_;
};
}  // namespace sim
}  // namespace gui
}  // namespace tobas
