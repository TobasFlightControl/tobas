// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QObject>
#include <QTimer>
#include <QWidget>

#include <tobas_rqt_bridge/bridge.hpp>

namespace tobas
{
namespace gui
{
namespace gcs
{
class NetworkChecker : public QObject
{
  Q_OBJECT

  using self = NetworkChecker;
  using super = QObject;

  static constexpr int kFirstTimeout = 5000;        // First timeout after communication is lost [ms].
  static constexpr int kSubsequentTimeout = 30000;  // Second and later timeouts after the first warning [ms].

public:
  explicit NetworkChecker(QWidget* parent, const RosQtBridge& bridge);

private:
  QWidget* const parent_;

  QTimer timeout_timer_;

private Q_SLOTS:
  void heartbeatCb(const tobas_msgs::msg::Heartbeat::ConstSharedPtr& msg);
  void onTimeout();
};
}  // namespace gcs
}  // namespace gui
}  // namespace tobas
