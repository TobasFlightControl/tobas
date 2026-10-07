// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_gcs/flight_controller_scanner.hpp"

#include <QMap>
#include <QSet>

namespace tobas
{
namespace gui
{
namespace gcs
{
namespace
{
constexpr char kServiceType[] = "_tobas-fc._tcp";

QMap<QString, QString> parseTxtField(const QString& txt)
{
  QMap<QString, QString> res;

  for (const auto& record : txt.split('"', Qt::SkipEmptyParts)) {
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

std::optional<QString> getOptionalValue(const QMap<QString, QString>& map, const QString& key)
{
  const auto it = map.find(key);
  if (it == map.end()) {
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

QVector<DiscoveredFlightController> parseAvahiBrowseResult(const QString& output)
{
  QVector<DiscoveredFlightController> flight_controllers;
  QSet<QString> addresses;

  for (const auto& line : output.split('\n', Qt::SkipEmptyParts)) {
    const auto fields = line.split(';');
    if (fields.size() < 10) {
      continue;
    }

    const auto& event_type = fields.at(0);
    if (event_type != "=") {
      continue;
    }

    const auto& protocol = fields.at(2);
    if (protocol != "IPv4") {
      continue;
    }

    const auto& service_type = fields.at(4);
    if (service_type != kServiceType) {
      continue;
    }

    const auto& hostname = fields.at(6);
    if (hostname.isEmpty()) {
      continue;
    }

    const auto& address = fields.at(7);
    if (address.isEmpty() || addresses.contains(address)) {
      continue;
    }

    const auto txt = parseTxtField(fields.at(9));
    flight_controllers.append({ hostname, address, getOptionalValue(txt, "drone"), getOptionalValue(txt, "id") });

    addresses.insert(address);
  }

  return flight_controllers;
}
}  // namespace

FlightControllerScanner::FlightControllerScanner(QObject* parent) : super(parent)
{
  constexpr int kScanInterval = 5000;  // [ms]
  scan_timer_.setInterval(kScanInterval);

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
  if (process_.state() != QProcess::NotRunning) {
    return;
  }

  failure_reported_ = false;
  process_.start("avahi-browse", { "--parsable", "--resolve", "--terminate", QString::fromUtf8(kServiceType) });
}

void FlightControllerScanner::onFinished(int exit_code, QProcess::ExitStatus exit_status)
{
  if (failure_reported_) {
    return;
  }

  if (exit_code != 0 || exit_status != QProcess::NormalExit) {
    auto message = QString::fromUtf8(process_.readAllStandardError()).trimmed();
    if (message.isEmpty()) {
      message = "avahi-browse exited unexpectedly.";
    }
    Q_EMIT failed(message);
    return;
  }

  Q_EMIT finished(parseAvahiBrowseResult(QString::fromUtf8(process_.readAllStandardOutput())));
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
}  // namespace gcs
}  // namespace gui
}  // namespace tobas
