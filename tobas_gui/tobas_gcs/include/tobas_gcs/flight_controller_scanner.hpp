// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <optional>

#include <QProcess>
#include <QString>
#include <QTimer>
#include <QVector>

namespace tobas::gui::gcs
{
struct DiscoveredFlightController
{
  QString hostname;
  QString address;
  std::optional<QString> drone;
  std::optional<QString> id;
};

class FlightControllerScanner : public QObject
{
  Q_OBJECT

  using self = FlightControllerScanner;
  using super = QObject;

Q_SIGNALS:
  void finished(const QVector<DiscoveredFlightController>& _flight_controllers);
  void failed(const QString& _message);

public:
  explicit FlightControllerScanner(QObject* _parent);

  void start();
  void stop();

private:
  QProcess process_;
  QTimer scan_timer_;
  bool failure_reported_ = false;

private Q_SLOTS:
  void scanOnce();
  void onFinished(int _exit_code, QProcess::ExitStatus _exit_status);
  void onErrorOccurred(QProcess::ProcessError _error);
};
}  // namespace tobas::gui::gcs
