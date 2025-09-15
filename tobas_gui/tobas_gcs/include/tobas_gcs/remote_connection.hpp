#pragma once

#include <QLabel>
#include <QPixmap>
#include <QTimer>
#include <QWidget>

#include <tobas_rqt_bridge/bridge.hpp>

namespace gui
{
namespace gcs
{
class RemoteConnectionWidget : public QWidget
{
  Q_OBJECT

  using self = RemoteConnectionWidget;
  using super = QWidget;

  static constexpr int kWidgetWidth = 120;
  static constexpr int kIconHeight = 40;

  static constexpr int kTimeout = 5000;  // [ms]

public:
  explicit RemoteConnectionWidget(const RosQtBridge& bridge);

  void start();
  void stop();

  void setConnected();
  void setDisonnected();
  void setUnknown();

private:
  const RosQtBridge& bridge_;

  QPixmap connected_;
  QPixmap disconnected_;
  QPixmap unknown_;

  QLabel* icon_;
  QLabel* label_;

  QTimer timeout_timer_;

  QMetaObject::Connection heartbeat_conn_;

  bool is_running_ = false;

  void setIconPixmap(const QPixmap& pixmap);

private Q_SLOTS:
  void heartbeatCb(const tobas_msgs::msg::Heartbeat::ConstSharedPtr& msg);
  void onTimeout();
};
}  // namespace gcs
}  // namespace gui
