#pragma once

#include <QtWidgets>

namespace tobas_gcs
{
class ConnectionChecker : public QObject
{
  Q_OBJECT

public:
  explicit ConnectionChecker(QObject* parent, size_t interval_ms);

  void setHostAddress(const QString& host_address);
  void setPort(uint16_t port);

private:
  QTimer timer_;
  bool is_connected_ = false;

  QString host_address_ = "000.000.000.000";
  uint16_t port_ = 80;

Q_SIGNALS:
  void stateChanged(bool connected);

private Q_SLOTS:
  void checkConnectionTimerCb();
};
}  // namespace tobas_gcs
