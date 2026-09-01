// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QLabel>
#include <QPixmap>
#include <QTimer>
#include <QWidget>

#include <tobas_rqt_bridge/bridge.hpp>

namespace tobas
{
namespace gui
{
namespace gcs
{
class RemoteConnectionWidget : public QWidget
{
  Q_OBJECT

  using self = RemoteConnectionWidget;
  using super = QWidget;

Q_SIGNALS:
  void disconnected();

public:
  explicit RemoteConnectionWidget(const rqt::RosQtBridge& bridge);

  void start();
  void stop();
  void restart();

private:
  const rqt::RosQtBridge& bridge_;

  QPixmap pixmap_connected_;
  QPixmap pixmap_disconnected_;
  QPixmap pixmap_unknown_;

  QLabel* icon_;
  QLabel* label_;

  QTimer timeout_timer_;

  QMetaObject::Connection heartbeat_conn_;

  bool is_running_ = false;

  enum State
  {
    kNotRunning,
    kConnected,
    kDisonnected,
    kUnknown,
  } state_ = kNotRunning;

  void setEmpty();
  void setConnected();
  void setDisonnected();
  void setUnknown();

  void setIconPixmap(const QPixmap& pixmap);

private Q_SLOTS:
  void heartbeatCb(const tobas_msgs::msg::Heartbeat::ConstSharedPtr& msg);
  void onTimeout();
};
}  // namespace gcs
}  // namespace gui
}  // namespace tobas
