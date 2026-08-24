// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QWidget>
#include <rclcpp/node.hpp>

namespace tobas
{
namespace gui
{
namespace sc
{
class BaseMagCalibWidget : public QWidget
{
  Q_OBJECT

public:
  virtual void reset() = 0;
  virtual void initializeRosInterfaces(rclcpp::Node::SharedPtr node, const std::string& ns) = 0;
};
}  // namespace sc
}  // namespace gui
}  // namespace tobas
