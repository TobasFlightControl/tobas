// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QProcess>
#include <QTimer>

namespace tobas::gui::sim
{
/** Own the launch process and its descendants in a private process group. */
class SimulationProcess : public QProcess
{
  Q_OBJECT

  using self = SimulationProcess;

public:
  explicit SimulationProcess(QObject* _parent = nullptr);
  ~SimulationProcess() override;

  void requestStop();

private:
  qint64 process_group_ = 0;
  bool stopping_ = false;
  QTimer stop_timer_;

  void signalProcessGroup(int _signal);
  void cleanupProcessGroup();

private Q_SLOTS:
  void onProcessStarted();
  void onProcessFinished();
  void onStopTimeout();
};
}  // namespace tobas::gui::sim
