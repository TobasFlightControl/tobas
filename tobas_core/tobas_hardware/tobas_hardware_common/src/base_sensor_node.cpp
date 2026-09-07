// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_hardware_common/base_sensor_node.hpp"

namespace tobas
{
namespace hardware
{
BaseSensorNode::BaseSensorNode(const std::string& name, const rclcpp::NodeOptions& options)
  : super(name, nodeOptions_Default(options))
{
  start_ss_ = createService<Empty>(name + "/start_main_timer", &self::startMainTimerSrvCb, this);
  stop_ss_ = createService<Empty>(name + "/stop_main_timer", &self::stopMainTimerSrvCb, this);
}

void BaseSensorNode::startMainTimerSrvCb(const Empty::Request::ConstSharedPtr&, const Empty::Response::SharedPtr&)
{
  main_timer_->reset();
}

void BaseSensorNode::stopMainTimerSrvCb(const Empty::Request::ConstSharedPtr&, const Empty::Response::SharedPtr&)
{
  main_timer_->cancel();
}
}  // namespace hardware
}  // namespace tobas
