#include <QTcpSocket>

#include "../../include/tobas_gcs/utils/connection_checker.hpp"

#define WAIT_FOR_CONNECT 1000  // [ms]

namespace tobas_gcs
{
ConnectionChecker::ConnectionChecker(QObject* parent, size_t interval_ms) : QObject(parent)
{
  connect(&timer_, &QTimer::timeout, this, &ConnectionChecker::checkConnectionTimerCb);
  timer_.start(interval_ms);  // ミリ秒単位で間隔を設定
}

void ConnectionChecker::setHostAddress(const QString& host_address)
{
  host_address_ = host_address;
}

void ConnectionChecker::setPort(uint16_t port)
{
  port_ = port;
}

void ConnectionChecker::checkConnectionTimerCb()
{
  QTcpSocket socket;  // 接続の切断を確認するために，ソケットオブジェクトを毎回作り直す
  socket.connectToHost(host_address_, port_);
  const auto is_connected_new = socket.waitForConnected(WAIT_FOR_CONNECT);

  if (is_connected_ != is_connected_new)
  {
    Q_EMIT stateChanged(is_connected_new);
    is_connected_ = is_connected_new;
  }
}
}  // namespace tobas_gcs
