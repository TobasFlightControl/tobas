// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_qt_tools/widgets/table_widget.hpp>
#include <tobas_rqt_bridge/bridge.hpp>

namespace tobas::gui::ctrl
{
class ConsoleWidget : public qt::TableWidget
{
  Q_OBJECT

  using self = ConsoleWidget;
  using super = qt::TableWidget;

public:
  explicit ConsoleWidget(const rqt::RosQtBridge& _bridge);

private Q_SLOTS:
  void messageCb(const tobas_msgs::msg::Message::ConstSharedPtr& _msg);
};
}  // namespace tobas::gui::ctrl
