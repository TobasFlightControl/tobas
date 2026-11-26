// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <optional>

#include <QElapsedTimer>
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

  /**
   * Start or restart periodic scanning and request an immediate scan.
   * An active scan continues until completion or timeout.
   */
  void reset();

  /** Stop periodic scanning and kill active discovery processes, discarding their results. */
  void cancel();

private:
  void killProcesses();

  QProcess process_;
  QTimer scan_timer_;
  QElapsedTimer scan_elapsed_;
  bool scan_discarded_ = false;
  int pending_resolutions_ = 0;

private Q_SLOTS:
  void scanOnce();
  void onScanFinished(int _exit_code, QProcess::ExitStatus _exit_status);
  void onScanFailed(QProcess::ProcessError _error);
};
}  // namespace tobas::gui::gcs
