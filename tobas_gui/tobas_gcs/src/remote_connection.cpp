// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_gcs/remote_connection.hpp"

#include <QVBoxLayout>

#include <tobas_qt_tools/event.hpp>
#include <tobas_qt_tools/util.hpp>

#include "tobas_gcs/util.hpp"

namespace tobas
{
namespace gui
{
namespace gcs
{
RemoteConnectionWidget::RemoteConnectionWidget(const rqt::RosQtBridge& bridge) : bridge_(bridge)
{
  const auto rsrc_dir = getResourceDir() / "connection";
  pixmap_connected_ = QPixmap(QString::fromStdString(rsrc_dir / "connected.png"));
  pixmap_disconnected_ = QPixmap(QString::fromStdString(rsrc_dir / "disconnected.png"));
  pixmap_unknown_ = QPixmap(QString::fromStdString(rsrc_dir / "unknown.png"));

  icon_ = new QLabel();
  icon_->setFixedSize(120, 40);          // Fix the size so the pixmap is not displayed too large.
  icon_->setAlignment(Qt::AlignCenter);  // Place the `QPixmap` at the center of the `QLabel`.

  label_ = new QLabel();

  constexpr int kTimeout = 10000;  // [ms]
  timeout_timer_.setInterval(kTimeout);

  setEmpty();

  // Layout
  const auto rows = new QVBoxLayout();
  qt::addWidgetCenter(icon_, rows);
  qt::addWidgetCenter(label_, rows);
  setLayout(rows);

  // Connection
  connect(&timeout_timer_, &QTimer::timeout, this, &self::onTimeout);
}

void RemoteConnectionWidget::start()
{
  if (is_running_) {
    return;
  }

  setUnknown();

  heartbeat_conn_ =
    connect(&bridge_, &rqt::RosQtBridge::remoteHeartbeatReceived, this, &self::heartbeatCb, Qt::QueuedConnection);
  timeout_timer_.start();

  is_running_ = true;
}

void RemoteConnectionWidget::stop()
{
  if (!is_running_) {
    return;
  }

  setEmpty();

  disconnect(heartbeat_conn_);
  timeout_timer_.stop();

  is_running_ = false;
}

void RemoteConnectionWidget::restart()
{
  if (!is_running_) {
    start();
  }

  stop();
  qt::processAllQueuedEvents();
  start();
}

void RemoteConnectionWidget::setEmpty()
{
  icon_->clear();
  label_->clear();
  state_ = kNotRunning;
}

void RemoteConnectionWidget::setConnected()
{
  setIconPixmap(pixmap_connected_);
  label_->setText("TELEM OK");
  state_ = kConnected;
}

void RemoteConnectionWidget::setDisonnected()
{
  setIconPixmap(pixmap_disconnected_);
  label_->setText("TELEM LOST");
  state_ = kDisonnected;
}

void RemoteConnectionWidget::setUnknown()
{
  setIconPixmap(pixmap_unknown_);
  label_->setText("Waiting...");
  state_ = kUnknown;
}

void RemoteConnectionWidget::setIconPixmap(const QPixmap& pixmap)
{
  icon_->setPixmap(pixmap.scaled(icon_->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
}

void RemoteConnectionWidget::heartbeatCb(const tobas_msgs::msg::Heartbeat::ConstSharedPtr&)
{
  setConnected();
  timeout_timer_.start();
}

void RemoteConnectionWidget::onTimeout()
{
  if (state_ == kConnected) {
    Q_EMIT disconnected();
  }
  setDisonnected();
}
}  // namespace gcs
}  // namespace gui
}  // namespace tobas
