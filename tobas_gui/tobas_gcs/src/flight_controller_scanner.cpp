// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_gcs/flight_controller_scanner.hpp"

#include <QMap>
#include <QSet>
#include <QVariant>

namespace tobas::gui::gcs
{
namespace
{
constexpr char kServiceType[] = "_tobas-fc._tcp";

QMap<QString, QString> parseTxtField(const QString& _txt)
{
  QMap<QString, QString> res;

  for (const auto& record : _txt.split('"', Qt::SkipEmptyParts)) {
    const auto separator = record.indexOf('=');
    if (separator < 0) {
      continue;
    }
    const auto key = record.left(separator).toLower();
    const auto value = record.mid(separator + 1);
    res.insert(key, value);
  }

  return res;
}

std::optional<QString> getOptionalValue(const QMap<QString, QString>& _map, const QString& _key)
{
  const auto it = _map.find(_key);
  if (it == _map.end()) {
    return std::nullopt;
  }
  else {
    const auto& value = it.value();
    if (value.isEmpty()) {
      return std::nullopt;
    }
    else {
      return value;
    }
  }
}

QVector<DiscoveredFlightController> parseAvahiBrowseResult(const QString& _output)
{
  QVector<DiscoveredFlightController> res;
  QSet<QString> hostnames, addresses;

  for (const auto& line : _output.split('\n', Qt::SkipEmptyParts)) {
    // Resolved entries: =;interface;protocol;name;type;domain;hostname;address;port;TXT.
    const auto fields = line.split(';');
    if (fields.size() < 10) {
      continue;
    }

    const auto& event_type = fields.at(0);
    if (event_type != "=") {
      continue;
    }

    // This filters the mDNS transport only; the address field may still contain IPv6.
    const auto& protocol = fields.at(2);
    if (protocol != "IPv4") {
      continue;
    }

    const auto& service_type = fields.at(4);
    if (service_type != kServiceType) {
      continue;
    }

    const auto& hostname = fields.at(6);
    if (hostname.isEmpty() || hostnames.contains(hostname)) {
      continue;
    }

    const auto& address = fields.at(7);
    if (address.isEmpty() || addresses.contains(address)) {
      continue;
    }

    const auto txt = parseTxtField(fields.at(9));
    res.append({ hostname, address, getOptionalValue(txt, "drone"), getOptionalValue(txt, "id") });

    hostnames.insert(hostname);
    addresses.insert(address);
  }

  return res;
}
}  // namespace

FlightControllerScanner::FlightControllerScanner(QObject* _parent) : super(_parent)
{
  constexpr int kScanInterval = 5000;  // [ms]
  scan_timer_.setInterval(kScanInterval);

  // These callbacks run on the scanner's thread; the external processes run asynchronously.
  connect(&scan_timer_, &QTimer::timeout, this, &self::scanOnce);
  connect(&process_, qOverload<int, QProcess::ExitStatus>(&QProcess::finished), this, &self::onFinished);
  connect(&process_, &QProcess::errorOccurred, this, &self::onErrorOccurred);
}

void FlightControllerScanner::start()
{
  scan_timer_.start();
  scanOnce();
}

void FlightControllerScanner::stop()
{
  scan_timer_.stop();
}

void FlightControllerScanner::scanOnce()
{
  // Browsing may have exited while IPv4 lookups are still pending. Wait for the entire scan to finish.
  if (process_.state() != QProcess::NotRunning || pending_resolutions_ > 0) {
    return;
  }

  failure_reported_ = false;
  process_.start("avahi-browse", { "--parsable", "--resolve", "--terminate", kServiceType });
}

void FlightControllerScanner::onFinished(int _exit_code, QProcess::ExitStatus _exit_status)
{
  if (failure_reported_) {
    return;
  }

  if (_exit_code != 0 || _exit_status != QProcess::NormalExit) {
    auto message = QString::fromUtf8(process_.readAllStandardError()).trimmed();
    if (message.isEmpty()) {
      message = "avahi-browse exited unexpectedly.";
    }
    Q_EMIT failed(message);
    return;
  }

  controllers_ = parseAvahiBrowseResult(QString::fromUtf8(process_.readAllStandardOutput()));
  if (controllers_.isEmpty()) {
    Q_EMIT finished(controllers_);
    return;
  }

  // Initialize the total before starting any process, including ones that may fail to start immediately.
  const auto controller_count = controllers_.size();
  pending_resolutions_ = controller_count;

  // avahi-browse cannot constrain the resolved address family. Request each hostname's A record with -4.
  for (qsizetype index = 0; index < controller_count; ++index) {
    auto* resolver = new QProcess(this);
    // Keep indices stable until every lookup has completed, regardless of completion order.
    resolver->setProperty("controllerIndex", index);
    connect(resolver, qOverload<int, QProcess::ExitStatus>(&QProcess::finished), this, &self::onResolverFinished);
    connect(resolver, &QProcess::errorOccurred, this, &self::onResolverErrorOccurred);
    resolver->start("avahi-resolve-host-name", { "-4", controllers_.at(index).hostname });
  }
}

void FlightControllerScanner::completeResolution(QProcess* _resolver, bool _success)
{
  auto& controller = controllers_[_resolver->property("controllerIndex").value<qsizetype>()];
  controller.address.clear();
  if (_success) {
    // The resolver prints "hostname\tIPv4 address". Failed lookups contribute no result.
    const auto fields = QString::fromUtf8(_resolver->readAllStandardOutput()).trimmed().split('\t');
    if (fields.size() == 2) {
      controller.address = fields.at(1);
    }
  }
  _resolver->deleteLater();

  if (--pending_resolutions_ == 0) {
    // Remove failed lookups and duplicate IPv4 addresses only after all stored indices are no longer needed.
    QSet<QString> addresses;
    controllers_.removeIf(
      [&addresses](const auto& _controller)
      {
        if (_controller.address.isEmpty() || addresses.contains(_controller.address)) {
          return true;
        }
        addresses.insert(_controller.address);
        return false;
      });
    Q_EMIT finished(controllers_);
  }
}

void FlightControllerScanner::onResolverFinished(int _exit_code, QProcess::ExitStatus _exit_status)
{
  completeResolution(qobject_cast<QProcess*>(sender()), _exit_code == 0 && _exit_status == QProcess::NormalExit);
}

void FlightControllerScanner::onResolverErrorOccurred(QProcess::ProcessError _error)
{
  // FailedToStart does not emit finished(). Other errors are handled there to avoid double counting.
  if (_error == QProcess::FailedToStart) {
    completeResolution(qobject_cast<QProcess*>(sender()), false);
  }
}

void FlightControllerScanner::onErrorOccurred(QProcess::ProcessError)
{
  failure_reported_ = true;
  auto message = process_.errorString();
  if (process_.error() == QProcess::FailedToStart) {
    message = "Failed to start avahi-browse. Install the avahi-utils package.";
  }
  Q_EMIT failed(message);
}
}  // namespace tobas::gui::gcs
