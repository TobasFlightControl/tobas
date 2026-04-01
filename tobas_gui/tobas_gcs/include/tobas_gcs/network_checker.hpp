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

  static constexpr int kFirstTimeout = 5000;        // 通信切断後の最初のタイムアウト [ms]
  static constexpr int kSubsequentTimeout = 30000;  // 1度警告後の2度目以降のタイムアウト [ms]

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
