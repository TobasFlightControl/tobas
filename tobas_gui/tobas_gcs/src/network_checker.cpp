#include "tobas_gcs/network_checker.hpp"

#include <QApplication>

#include <tobas_qt_tools/message.hpp>

namespace gui
{
namespace gcs
{
NetworkChecker::NetworkChecker(QWidget* parent, const RosQtBridge& bridge) : parent_(parent)
{
  connect(&bridge, &RosQtBridge::localHeartbeatReceived, this, &self::heartbeatCb, Qt::QueuedConnection);
  connect(&timeout_timer_, &QTimer::timeout, this, &self::onTimeout);
}

void NetworkChecker::heartbeatCb(const tobas_msgs::msg::Heartbeat::ConstSharedPtr&)
{
  timeout_timer_.start(kFirstTimeout);
}

void NetworkChecker::onTimeout()
{
  if (qt::yesOrNo(parent_, "Network connection was lost. Do you want to close the application?", qt::WARN)) {
    QApplication::quit();
  }

  timeout_timer_.start(kSubsequentTimeout);
}
}  // namespace gcs
}  // namespace gui
