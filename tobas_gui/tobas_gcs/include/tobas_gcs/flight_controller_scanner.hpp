// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QProcess>
#include <QString>
#include <QVector>

namespace tobas
{
namespace gui
{
namespace gcs
{
struct DiscoveredFlightController
{
  QString hostname;
  QString address;
};

class FlightControllerScanner : public QObject
{
  Q_OBJECT

  using self = FlightControllerScanner;
  using super = QObject;

Q_SIGNALS:
  void finished(const QVector<DiscoveredFlightController>& flight_controllers);
  void failed(const QString& message);

public:
  explicit FlightControllerScanner(QObject* parent);

  void start();

private:
  QProcess process_;
  bool failure_reported_ = false;

private Q_SLOTS:
  void onFinished(int exit_code, QProcess::ExitStatus exit_status);
  void onErrorOccurred(QProcess::ProcessError error);
};
}  // namespace gcs
}  // namespace gui
}  // namespace tobas
