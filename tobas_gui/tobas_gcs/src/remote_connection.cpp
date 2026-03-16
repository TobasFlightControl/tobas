#include "tobas_gcs/remote_connection.hpp"

#include <QVBoxLayout>

#include <tobas_qt_tools/event.hpp>
#include <tobas_qt_tools/util.hpp>

#include "tobas_gcs/util.hpp"

namespace gui
{
namespace gcs
{
RemoteConnectionWidget::RemoteConnectionWidget(const RosQtBridge& bridge) : bridge_(bridge)
{
  const auto rsrc_dir = getResourceDir() / "connection";
  connected_ = QPixmap(QString::fromStdString(rsrc_dir / "connected.png"));
  disconnected_ = QPixmap(QString::fromStdString(rsrc_dir / "disconnected.png"));
  unknown_ = QPixmap(QString::fromStdString(rsrc_dir / "unknown.png"));

  icon_ = new QLabel();
  icon_->setFixedSize(120, 40);  // サイズを固定しておかないとピクセルマップが大きく表示されてしまう
  icon_->setAlignment(Qt::AlignCenter);  // QPixmapをQLabelの中央に配置

  label_ = new QLabel();

  timeout_timer_.setInterval(5000);

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
    connect(&bridge_, &RosQtBridge::remoteHeartbeatReceived, this, &self::heartbeatCb, Qt::QueuedConnection);
  timeout_timer_.start();

  is_running_ = true;
}

void RemoteConnectionWidget::stop()
{
  if (!is_running_) {
    return;
  }

  setUnknown();

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

void RemoteConnectionWidget::setConnected()
{
  state_ = kConnected;
  setIconPixmap(connected_);
  label_->setText("Connected");
}

void RemoteConnectionWidget::setDisonnected()
{
  state_ = kDisonnected;
  setIconPixmap(disconnected_);
  label_->setText("Not Connected");
}

void RemoteConnectionWidget::setUnknown()
{
  state_ = kUnknown;
  setIconPixmap(unknown_);
  label_->setText("Connecting...");
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
