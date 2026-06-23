// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "./ros_interface.hpp"

namespace tobas
{
RosInterfaceNode::RosInterfaceNode(const rclcpp::NodeOptions& options)
  : super("ros_interface", nodeOptions_Default(options))
{
  // サービスコールバックを再帰的に呼んだ際のデッドロックを回避
  // cf. https://answers.ros.org/question/343279/ros2-how-to-implement-a-sync-service-client-in-a-node/
  group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  // ROSインターフェースを登録
  // メモリ削減のために分割コンパイルするためにメソッドを分割している
  registerTopicsLogicToIface();
  registerTopicsIfaceToLogic();
  registerServices();
  registerActions();
}
}  // namespace tobas
