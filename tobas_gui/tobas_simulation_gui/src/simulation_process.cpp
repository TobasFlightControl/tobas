// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_simulation_gui/simulation_process.hpp"

#include <cerrno>
#include <csignal>
#include <cstring>

#include <QDebug>

namespace tobas::gui::sim
{
SimulationProcess::SimulationProcess(QObject* _parent) : QProcess(_parent)
{
  // Use a private process group to signal only the launched processes without affecting GCS or other simulations.
  setUnixProcessParameters(QProcess::UnixProcessFlag::CreateNewSession);

  stop_timer_.setSingleShot(true);

  connect(this, &QProcess::started, this, &self::onProcessStarted);
  connect(this, qOverload<int, QProcess::ExitStatus>(&QProcess::finished), this, &self::onProcessFinished);
  connect(&stop_timer_, &QTimer::timeout, this, &self::onStopTimeout);
}

SimulationProcess::~SimulationProcess()
{
  // Wait for shutdown without relying on the timer, since the event loop may have stopped during destruction.
  requestStop();

  // Wait for the process to finish.
  if (state() != QProcess::NotRunning) {
    qInfo() << "Waiting for the simulation process to finish.";
    waitForFinished(5000);
  }

  // Clean up any child processes that remain after the launch process exits.
  cleanupProcessGroup();
}

void SimulationProcess::requestStop()
{
  // Ignore duplicate requests to avoid extending the grace period. Do nothing if the process has already stopped.
  if (state() == QProcess::NotRunning || stopping_) {
    return;
  }

  stopping_ = true;

  // Signal immediately if startup is complete. Handle requests made during startup in `onProcessStarted`.
  if (process_group_ > 0) {
    signalProcessGroup(SIGINT);
    stop_timer_.start(5000);
  }
}

void SimulationProcess::signalProcessGroup(int _signal)
{
  // Only use positive IDs, since `killpg(0, ...)` would signal GCS's own process group before the ID is available.
  if (process_group_ <= 0) {
    return;
  }

  qInfo() << "Sending signal" << _signal << "to simulation process group" << process_group_;

  // Also reach Gazebo when launched through a shell. Ignore ESRCH if the process group has already exited.
  if (::killpg(process_group_, _signal) != 0 && errno != ESRCH) {
    qWarning() << "Failed to signal the simulation process group:" << std::strerror(errno);
  }
}

void SimulationProcess::cleanupProcessGroup()
{
  // Prevent a delayed timeout from affecting the next launch.
  stop_timer_.stop();

  // Prevent orphaned child processes even if the launch process exits abnormally.
  signalProcessGroup(SIGKILL);

  // Reset the process management state.
  process_group_ = 0;
  stopping_ = false;
}

void SimulationProcess::onProcessStarted()
{
  process_group_ = processId();

  if (stopping_) {
    signalProcessGroup(SIGINT);
    stop_timer_.start(5000);
  }
}

void SimulationProcess::onProcessFinished()
{
  cleanupProcessGroup();
}

void SimulationProcess::onStopTimeout()
{
  // Force the entire process group to exit if SIGINT does not stop it.
  qWarning() << "Simulation did not stop after SIGINT; sending SIGKILL.";
  signalProcessGroup(SIGKILL);
}
}  // namespace tobas::gui::sim
