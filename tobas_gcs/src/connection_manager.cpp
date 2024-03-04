#include <ros/ros.h>

#include "../include/tobas_gcs/connection_manager.hpp"

#define CHECK_INTERVAL 1000  // [ms]
#define PORT 80

namespace tobas_gcs
{
ConnectionStatus::ConnectionStatus(QWidget* parent) : QLabel(parent)
{
  setConnected(false);
}

void ConnectionStatus::setConnected(bool connected)
{
  if (connected)
    setText("Connected");
  else
    setText("Disconnected");
}

ConnectionManager::ConnectionManager(QWidget* parent)
  : QWidget(parent),
    checker_(new ConnectionChecker(this, CHECK_INTERVAL)),
    ip_getter_(new IpAddressGetter(this)),
    status_(new ConnectionStatus(this))
{
  auto* cols = new QHBoxLayout(this);
  setLayout(cols);

  cols->addWidget(new QLabel("IP:", this));

  cols->addWidget(ip_getter_);
  ip_getter_->setIP(192, 168, 249, 1);

  cols->addWidget(status_);

  checker_->setHostAddress(ip_getter_->getIP());
  checker_->setPort(PORT);

  connect(
    checker_, &ConnectionChecker::stateChanged, this,
    &ConnectionManager::onConnectionCheckerStateChanged);
  connect(
    ip_getter_, &IpAddressGetter::ipChanged, this, &ConnectionManager::onIpAddressGetterIpChanged);
}

void ConnectionManager::onConnectionCheckerStateChanged(bool connected)
{
  ROS_DEBUG_STREAM("ConnectionManager::onConnectionCheckerStateChanged(" << connected << ")");

  status_->setConnected(connected);
}

void ConnectionManager::onIpAddressGetterIpChanged(const QString& ip)
{
  ROS_DEBUG_STREAM("ConnectionManager::onIpAddressGetterIpChanged(" << ip.toStdString() << ")");

  checker_->setHostAddress(ip);
}
}  // namespace tobas_gcs
