// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_gcs/flight_controller_scanner.hpp"

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

QVector<DiscoveredFlightController> parse(const QString& output)
{
  QVector<DiscoveredFlightController> flight_controllers;
  QSet<QString> addresses;

  for (const auto& line : output.split('\n', Qt::SkipEmptyParts)) {
    const auto fields = line.split(';');
    if (fields.size() < 9 || fields.at(0) != "=" || fields.at(2) != "IPv4" || fields.at(4) != kServiceType) {
      continue;
    }

    const auto& hostname = fields.at(6);
    const auto& address = fields.at(7);

    if (hostname.isEmpty() || address.isEmpty() || addresses.contains(address)) {
      continue;
    }

    addresses.insert(address);
    flight_controllers.append({ hostname, address });
  }

  return flight_controllers;
}
}  // namespace

FlightControllerScanner::FlightControllerScanner(QObject* parent) : super(parent), process_(this)
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

  Q_EMIT finished(parse(QString::fromUtf8(process_.readAllStandardOutput())));
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
