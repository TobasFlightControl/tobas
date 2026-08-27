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
#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/path.hpp>
#include <tobas_qt_tools/string.hpp>
#include <tobas_qt_tools/util.hpp>
#include <tobas_qt_tools/widgets/progress_dialog.hpp>
#include <tobas_std_tools/check.hpp>

#include "tobas_simulation_gui/gazebo.hpp"

using namespace std::chrono_literals;
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
  dynamic_config_->reset();
  commanders_->reset();

  if (launch_proc_) {
    terminateSimulation();
    qWarning() << "Gazebo was forcibly shut down.";
  }

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
  node_ = std::move(node);

  dynamic_config_->initializeRosInterfaces(node_, ns);
  dynamic_config_->setEnabled(true);

  commanders_->initializeRosInterfaces(node_, ns);
  commanders_->setEnabled(true);
}

void SimulationWidget::clearRosInterfaces()
{
  node_.reset();

  dynamic_config_->clearRosInterfaces();
  dynamic_config_->setEnabled(false);

  commanders_->clearRosInterfaces();
  commanders_->setEnabled(false);
}

bool SimulationWidget::isRunning() const
{
  return launch_proc_ && launch_proc_->state() == QProcess::Running;
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

std::map<QString, QString> SimulationWidget::makeGazeboLaunchArguments(bool launch_core) const
{
  std::map<QString, QString> args{
    { "user_debug", qt::boolToText(sim_settings_->userDebug()) },
    { "id", "id" + QString::number(kDroneId) },
    { "launch_core", qt::boolToText(launch_core) },
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

void SimulationWidget::launchSimulation(bool launch_core)
{
  const auto args = makeGazeboLaunchArguments(launch_core);

  QStringList command = { "launch", QString::fromStdString(proj_paths_.cfgPkgName()), "gazebo.launch.xml" };
  for (const auto& [arg_name, arg_value] : args) {
    command.append(arg_name + ":=" + arg_value);
  }

  launch_proc_ = new QProcess(this);
  launch_proc_->setProcessChannelMode(QProcess::ForwardedChannels);  // Forward child process output to the caller.
  connect(launch_proc_, qOverload<int, QProcess::ExitStatus>(&QProcess::finished), this, &self::onLaunchProcessFinished);
  launch_proc_->start("ros2", command);
}

void SimulationWidget::terminateLaunchProcess()
{
  if (!launch_proc_) {
    qInfo() << "The roslaunch process is null.";
    return;
  }

  if (launch_proc_->state() != QProcess::Running) {
    qInfo() << "The roslaunch process is not running.";
    launch_proc_->deleteLater();
    return;
  }

  // Get the process ID.
  const auto pid = launch_proc_->processId();

  // Send SIGINT to the process.
  // If the parent process is forcibly stopped with `QProcess::terminate`, child processes such as nodes do not exit.
  qInfo() << "Sending SIGINT to the launch process.";
  if (kill(pid, SIGINT) != 0) {
    if (errno == ESRCH) {
      throw std::runtime_error("PID " + std::to_string(pid) + " not found.");
    }
    else {
      throw std::runtime_error("Failed to kill PID " + std::to_string(pid) + ": " + linux::strError());
    }
  }

  // Reset `QProcess`.
  launch_proc_->deleteLater();
}

void SimulationWidget::terminateSimulation()
{
  clearRosInterfaces();
  terminateLaunchProcess();
  killGazeboServer();
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
  launchSimulation(true);
  if (!waitUntilGazeboServerReady()) {
    progress.close();
    if (launch_proc_) {
      qt::qErrorBox(this, "Failed to start the Gazebo server.");
    }
    reset();
    return;
  }
  progress.progressStep();

  // Wait for Gazebo rendering to start.
  progress.setLabelText("Waiting for Gazebo rendering to start.");
  if (!waitUntilGazeboRenderingReady()) {
    progress.close();
    if (launch_proc_) {
      qt::qErrorBox(this, "Failed to get the Gazebo rendering information.");
    }
    reset();
    return;
  }
  progress.progressStep();

  // Close the progress bar.
  progress.close();

  sim_settings_->setEnabled(false);

  Q_EMIT started();
  qt::qInfoBox(this, "The simulation has started successfully.");
}

void SimulationWidget::onTerminateRequested()
{
  qDebug() << "SimulationWidget::onTerminateRequested";

  Q_EMIT telemetryLossExpected();

  // Stop the Gazebo process on another thread.
  terminateSimulation();
  spinner_.start();
  qInfo() << "Waiting for Gazebo to shutdown.";
  while (!waitUntilGazeboShutdown(node_, 5s)) {
    qWarning() << "Failed to shutdown the Gazebo server. Trying again...";
  }
  spinner_.stop();

  qInfo() << "Restoring to the state before the simulation started.";
  reset();

  Q_EMIT terminated();
  qt::qInfoBox(this, "The simulation has been terminated successfully.");
}

void SimulationWidget::onLaunchProcessFinished(int code, QProcess::ExitStatus status)
{
  qDebug().nospace() << "SimulationWidget::onLaunchProcessFinished(" << code << ", " << status << ")";

  if (status == QProcess::CrashExit) {
    throw std::runtime_error("The simulation process crashed: " + launch_proc_->errorString().toStdString());
  }
  else if (code != 0) {
    // Get the output.
    const auto std_out = QString::fromLocal8Bit(launch_proc_->readAllStandardOutput());
    const auto std_err = QString::fromLocal8Bit(launch_proc_->readAllStandardError());

    // Destroy processes that have already exited.
    // Otherwise, if SIGINT is sent to an already-dead process from elsewhere,
    // it is promoted to SIGTERM after a few seconds and the entire GCS exits.
    launch_proc_->terminate();
    launch_proc_->deleteLater();

    // Kill any Gazebo server that may still remain.
    killGazeboServer();

    // Save the output.
    const auto out_msg = std_out + "\n\n" + std_err;
    const auto log_path = qt::writeTimestampedFile(out_msg + '\n', qt::expandUser(kGuiLogDir), "", "simulation_crash");

    // Show an error message.
    if (log_path) {
      qt::qErrorBox(
        this, "Simulation process was terminated unexpectedly. The output has been saved to:\n" + log_path.value());
    }
    else {
      qWarning() << "Failed to save the simulation crash output.";
      qt::qErrorBox(this, "Simulation process was terminated unexpectedly:\n\n" + out_msg);
    }

    // Initialize the entire widget.
    reset();
  }
}
}  // namespace sim
}  // namespace gui
}  // namespace tobas
