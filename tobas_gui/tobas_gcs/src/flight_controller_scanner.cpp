// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_gcs/flight_controller_scanner.hpp"

#include <algorithm>
#include <memory>

#include <QMap>
#include <QSet>

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

  const auto controllers = parseAvahiBrowseResult(QString::fromUtf8(process_.readAllStandardOutput()));
  if (controllers.isEmpty()) {
    Q_EMIT finished({});
    return;
  }

  // All callbacks share one result list and keep it alive after this function returns.
  const auto controllers_ipv4 = std::make_shared<QVector<DiscoveredFlightController>>();
  // Initialize the total before starting any process, including ones that may fail to start immediately.
  pending_resolutions_ = controllers.size();

  // avahi-browse cannot constrain the resolved address family. Request each hostname's A record with -4.
  for (const auto& fc : controllers) {
    auto* resolver = new QProcess(this);
    const auto complete = [this, resolver, fc, controllers_ipv4](int _code, QProcess::ExitStatus _status)
    {
      // The resolver prints "hostname\tIPv4 address". Failed lookups contribute no result.
      const auto fields = QString::fromUtf8(resolver->readAllStandardOutput()).trimmed().split('\t');
      if (_code == 0 && _status == QProcess::NormalExit && fields.size() == 2) {
        const auto& address_ipv4 = fields.at(1);
        // Keep the final FC list unique by IPv4 address.
        if (std::ranges::none_of(*controllers_ipv4, [&](const auto& _fc) { return _fc.address == address_ipv4; })) {
          controllers_ipv4->append({ fc.hostname, address_ipv4, fc.drone, fc.id });
        }
      }
      resolver->deleteLater();
      // Publish once, after every lookup has finished.
      if (--pending_resolutions_ == 0) {
        Q_EMIT finished(*controllers_ipv4);
      }
    };
    connect(resolver, qOverload<int, QProcess::ExitStatus>(&QProcess::finished), this, complete);
    connect(
      resolver,
      &QProcess::errorOccurred,
      this,
      [complete](QProcess::ProcessError _error)
      {
        // FailedToStart does not emit finished(). Other errors are handled there to avoid double counting.
        if (_error == QProcess::FailedToStart) {
          complete(-1, QProcess::CrashExit);
        }
      });
    resolver->start("avahi-resolve-host-name", { "-4", fc.hostname });
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
