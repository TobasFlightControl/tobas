// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_qt_tools/widgets/table_widget.hpp>
#include <tobas_rqt_bridge/bridge.hpp>

namespace tobas
{
namespace gui
{
namespace ctrl
{
class ConsoleWidget : public QWidget
{
  Q_OBJECT

  using self = ConsoleWidget;
  using super = QWidget;

public:
  explicit ConsoleWidget(const rqt::RosQtBridge& bridge);

  void reset();

private:
  qt::TableWidget* table_;

private Q_SLOTS:
  void messageCb(const tobas_msgs::msg::Message::ConstSharedPtr& msg);
};
}  // namespace ctrl
}  // namespace gui
}  // namespace tobas
