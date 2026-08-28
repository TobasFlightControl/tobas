// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_simulation_gui/simulation.hpp"

#include <csignal>

#include <QCloseEvent>
#include <QDebug>
#include <QHBoxLayout>
#include <QVBoxLayout>

#include <tobas_constants/path.hpp>
#include <tobas_gui_common/constants.hpp>
#include <tobas_gui_common/local_project_builder.hpp>
#include <tobas_linux/error.hpp>
#include <tobas_qt_tools/cast.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/path.hpp>
#include <tobas_qt_tools/string.hpp>
#include <tobas_qt_tools/util.hpp>
#include <tobas_qt_tools/widgets/progress_dialog.hpp>
#include <tobas_std_tools/check.hpp>

#include "tobas_simulation_gui/gazebo.hpp"

namespace fs = std::filesystem;

namespace tobas
{
namespace gui
{
namespace sim
{
SimulationWidget::SimulationWidget(const rqt::RosQtBridge& bridge) : spinner_(Qt::WindowModal, this)
{
  start_stop_button_ = new qt::ToggleButton("Start", "Terminate");
  start_stop_button_->setFixedSize(100, 40);

  sim_settings_ = new SimulationSettingsWidget();
  dynamic_config_ = new DynamicConfigWidget();
  commanders_ = new CommandersWidget(bridge, tree_, drone_);

  // Layout
  const auto config_rows = new QVBoxLayout();
  config_rows->addWidget(sim_settings_);
  qt::addWidgetCenter(start_stop_button_, config_rows);

  const auto cols = new QHBoxLayout();
  cols->addLayout(config_rows, 1);
  cols->addWidget(dynamic_config_, 1);
  cols->addWidget(commanders_, 1);

  setLayout(cols);

  // Connection
  connect(start_stop_button_, &qt::ToggleButton::checked, this, &self::onStartRequested);
  connect(start_stop_button_, &qt::ToggleButton::unchecked, this, &self::onTerminateRequested);

  reset();
}

void SimulationWidget::reset()
{
  TOBAS_CHECK(!launch_proc_);
  TOBAS_CHECK(state_ == kIdle);

  dynamic_config_->reset();
  commanders_->reset();

  spinner_.stop();
  start_stop_button_->setChecked(false);

  sim_settings_->setEnabled(project_loaded_);
  dynamic_config_->setEnabled(false);
  commanders_->setEnabled(false);
}

void SimulationWidget::updateProject(const fs::path& proj_path)
{
  // Update project path.
  proj_paths_.setProjPath(proj_path);

  // Load KDL tree.
  const auto uadf_path = proj_paths_.originalUadfPath();
  TOBAS_CHECK(uadf_parser_.parseFromPath(uadf_path, uadf_));
  TOBAS_CHECK(tree_parser_.parseFromUrdf(*uadf_.urdf, tree_));

  // Load drone configuration.
  const auto tbsdrn_path = proj_paths_.tbsdrnPath();
  TOBAS_CHECK(drone_.load(tbsdrn_path));

  commanders_->updateInternalDataStructures();

  project_loaded_ = true;
}

void SimulationWidget::initializeRosInterfaces(rclcpp::Node::SharedPtr node, const std::string& ns)
{
  dynamic_config_->initializeRosInterfaces(node, ns);
  dynamic_config_->setEnabled(true);

  commanders_->initializeRosInterfaces(node, ns);
  commanders_->setEnabled(true);
}

void SimulationWidget::clearRosInterfaces()
{
  dynamic_config_->clearRosInterfaces();
  dynamic_config_->setEnabled(false);

  commanders_->clearRosInterfaces();
  commanders_->setEnabled(false);
}

bool SimulationWidget::isRunning() const
{
  return state_ != kIdle;
}

void SimulationWidget::closeEvent(QCloseEvent* event)
{
  qDebug() << "SimulationWidget::closeEvent";

  // Destroy child processes when closing the parent widget.
  if (launch_proc_) {
    terminateSimulation();
  }

  event->accept();
}

std::map<QString, QString> SimulationWidget::makeGazeboLaunchArguments() const
{
  std::map<QString, QString> args{
    { "user_debug", qt::boolToText(sim_settings_->userDebug()) },
    { "id", "id" + QString::number(kDroneId) },
    { "x", QString::number(sim_settings_->x()) },
    { "y", QString::number(sim_settings_->y()) },
    { "z", QString::number(sim_settings_->z()) },
    { "roll", QString::number(sim_settings_->roll()) },
    { "pitch", QString::number(sim_settings_->pitch()) },
    { "yaw", QString::number(sim_settings_->yaw()) },
  };

  const auto world_path = sim_settings_->worldPath();
  if (!world_path.empty()) {
    args["world_path"] = QString::fromStdString(world_path);
  }

  const auto sbus_device = sim_settings_->sbusDevicePath();
  if (!sbus_device.empty()) {
    args["sbus_device"] = QString::fromStdString(sbus_device);
  }

  return args;
}

void SimulationWidget::launchSimulation()
{
  TOBAS_CHECK(!launch_proc_);
  TOBAS_CHECK(state_ == kIdle);

  const auto pkg_name = QString::fromStdString(proj_paths_.cfgPkgName());
  constexpr char kLaunchFileName[] = "gazebo.launch.xml";
  QStringList command = { "launch", pkg_name, kLaunchFileName };

  const auto args = makeGazeboLaunchArguments();
  for (const auto& [arg_name, arg_value] : args) {
    command.append(arg_name + ":=" + arg_value);
  }

  launch_proc_ = new QProcess(this);
  launch_proc_->setProcessChannelMode(QProcess::ForwardedChannels);  // Forward child process output to the caller.
  connect(launch_proc_, qOverload<int, QProcess::ExitStatus>(&QProcess::finished), this, &self::onLaunchProcessFinished);
  connect(launch_proc_, &QProcess::errorOccurred, this, &self::onLaunchProcessErrorOccurred);
  launch_proc_->start("ros2", command);

  state_ = kStarting;

  qInfo().nospace() << kLaunchFileName << " has been started with PID " << launch_proc_->processId() << ".";
}

void SimulationWidget::terminateLaunchProcess()
{
  TOBAS_CHECK(launch_proc_);
  TOBAS_CHECK(launch_proc_->state() != QProcess::NotRunning);

  // Get the process ID.
  const auto pid = launch_proc_->processId();
  TOBAS_CHECK(pid > 0);

  // Send SIGINT to the process.
  // If the parent process is forcibly stopped with `QProcess::terminate`, child processes such as nodes do not exit.
  qInfo().nospace() << "Sending SIGINT to the the simulation process (" << pid << ").";
  TOBAS_CHECK(kill(pid, SIGINT) == 0);
}

void SimulationWidget::terminateSimulation()
{
  TOBAS_CHECK(launch_proc_);
  TOBAS_CHECK(state_ != kIdle);

  if (state_ == kStopping) {
    qInfo() << "A simulation termination request has already been issued.";
    return;
  }

  terminateLaunchProcess();
  killGazeboServer();

  state_ = kStopping;
}

void SimulationWidget::onLaunchProcessFinished(int code, QProcess::ExitStatus status)
{
  qDebug().nospace() << "SimulationWidget::onLaunchProcessFinished(" << code << ", " << status << ")";

  const auto process = qt::qPointerCast<QProcess>(sender());
  finalizeLaunchProcess(process, code, status);
}

void SimulationWidget::finalizeLaunchProcess(QProcess* process, int code, QProcess::ExitStatus status)
{
  TOBAS_CHECK(process == launch_proc_);

  spinner_.stop();

  const bool expected = (state_ == kStopping);
  const auto error_string = process->errorString();
  const auto std_out = QString::fromLocal8Bit(process->readAllStandardOutput());
  const auto std_err = QString::fromLocal8Bit(process->readAllStandardError());

  // Clear the guarded pointer before resetting the widget to prevent duplicate termination requests.
  process->deleteLater();
  launch_proc_ = nullptr;
  state_ = kIdle;

  clearRosInterfaces();
  reset();

  Q_EMIT terminated();

  if (expected) {
    qt::qInfoBox(this, "The simulation has been terminated successfully.");
  }
  else {
    killGazeboServer();

    const auto out_msg = std_out + "\n\n" + std_err;
    const auto log_path = qt::writeTimestampedFile(out_msg + '\n', qt::expandUser(kGuiLogDir), "", "simulation_crash");

    QString error_msg = "Simulation process was terminated unexpectedly";
    if (status == QProcess::CrashExit) {
      error_msg += ":\n\n" + error_string;
    }
    else {
      error_msg += " with exit code " + QString::number(code) + ".";
    }
    if (log_path) {
      error_msg += "\n\nThe output has been saved to:\n" + log_path.value();
    }
    else if (!out_msg.trimmed().isEmpty()) {
      error_msg += "\n\n" + out_msg;
    }

    qt::qErrorBox(this, error_msg);
  }
}

void SimulationWidget::onStartRequested()
{
  qDebug() << "SimulationWidget::onStartRequested";

  // Create a progress bar.
  qt::ProgressDialog progress("Start Simulation", 3, this);
  progress.setCancelButton(nullptr);
  progress.show();

  // Build Tobas packages.
  progress.setLabelText("Building the Tobas project packages.");
  const auto build_res = cmn::buildLocalProject(proj_paths_.getProjPath());
  if (!build_res) {
    progress.close();
    const auto& error_msg = build_res.error();
    if (error_msg.size() < cmn::kSaveLogTextSizeThresh) {
      qt::qErrorBox(this, "Failed to build the Tobas project:\n\n" + error_msg);
    }
    else {
      const auto log_path =
        qt::writeTimestampedFile(error_msg + '\n', qt::expandUser(kGuiLogDir), "", "builderr_project_local");
      if (log_path) {
        qt::qErrorBox(this, "Failed to build the Tobas project. The output has been saved to:\n" + log_path.value());
      }
      else {
        qt::qErrorBox(this, "Failed to build the Tobas project, and also failed to save the error message.");
      }
    }
    reset();
    return;
  }
  progress.progressStep();

  // Launch Gazebo and wait for the server to start.
  progress.setLabelText("Launching the simulation.");
  launchSimulation();
  if (!waitUntilGazeboServerReady()) {
    progress.close();
    qt::qErrorBox(this, "Failed to start the Gazebo server.");
    reset();
    return;
  }
  progress.progressStep();

  // Wait for Gazebo rendering to start.
  progress.setLabelText("Waiting for Gazebo rendering to start.");
  if (!waitUntilGazeboRenderingReady()) {
    progress.close();
    qt::qErrorBox(this, "Failed to get the Gazebo rendering information.");
    reset();
    return;
  }
  progress.progressStep();

  // Close the progress bar.
  progress.close();

  sim_settings_->setEnabled(false);
  state_ = kRunning;
  Q_EMIT started();

  qt::qInfoBox(this, "The simulation has started successfully.");
}

void SimulationWidget::onTerminateRequested()
{
  qDebug() << "SimulationWidget::onTerminateRequested";

  TOBAS_CHECK(state_ == kRunning);

  Q_EMIT telemetryLossExpected();
  terminateSimulation();
  TOBAS_CHECK(state_ == kStopping);

  qInfo() << "Waiting for the simulation process to shutdown.";
  spinner_.start();
}

void SimulationWidget::onLaunchProcessErrorOccurred(QProcess::ProcessError error)
{
  qDebug().nospace() << "SimulationWidget::onLaunchProcessErrorOccurred(" << error << ")";

  if (error != QProcess::FailedToStart) {
    return;
  }

  const auto process = qt::qPointerCast<QProcess>(sender());
  finalizeLaunchProcess(process, -1, QProcess::CrashExit);
}
}  // namespace sim
}  // namespace gui
}  // namespace tobas
