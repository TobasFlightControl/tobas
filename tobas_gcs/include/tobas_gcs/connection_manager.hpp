#pragma once

#include <QtWidgets>

#include "./utils/ip_getter.hpp"
#include "./utils/connection_checker.hpp"

namespace tobas_gcs
{
class ConnectionStatus : public QLabel
{
  Q_OBJECT

public:
  explicit ConnectionStatus(QWidget* parent);

  void setConnected(bool connected);
};

class ConnectionManager : public QWidget
{
  Q_OBJECT

public:
  explicit ConnectionManager(QWidget* parent);

private:
  ConnectionChecker* checker_;
  IpAddressGetter* ip_getter_;
  ConnectionStatus* status_;

private Q_SLOTS:
  void onConnectionCheckerStateChanged(bool connected);
  void onIpAddressGetterIpChanged(const QString& ip);
};
}  // namespace tobas_gcs
