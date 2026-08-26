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
namespace
{
constexpr auto kWaitForServerTimeout = 3s;
}  // namespace

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
  connect(&bridge, &rqt::RosQtBridge::armingReceived, this, &self::armingCb, Qt::QueuedConnection);

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

  arming_.reset();

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

  ssh_client_.emplace(node_);
  remote_proj_builder_.emplace(node_);
  dynamic_config_->initializeRosInterfaces(node_, ns);
  commanders_->initializeRosInterfaces(node_, ns);
}

void SimulationWidget::clearRosInterfaces()
{
  node_.reset();

  ssh_client_.reset();
  remote_proj_builder_.reset();
  dynamic_config_->clearRosInterfaces();
  commanders_->clearRosInterfaces();
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

bool SimulationWidget::startSITL()
{
  // Confirm that the flight code is not running.
  if (arming_) {
    qt::qWarnBox(this, "This operation cannot be performed while flight controller is active.");
    return false;
  }

  // Create a progress bar.
  qt::ProgressDialog progress("Start SITL", 5, this);
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
    return false;
  }
  progress.progressStep();

  // Launch Gazebo and wait for the server to start.
  progress.setLabelText("Launching the simulation.");
  launchSimulation(true);
  if (!waitUntilGazeboServerReady()) {
    progress.close();
    if (launch_proc_) {
      qt::qErrorBox(this, "Failed to start the Gazebo server.");
      reset();
    }
    return false;
  }
  progress.progressStep();

  // Wait for Gazebo rendering to start.
  progress.setLabelText("Waiting for Gazebo rendering to start.");
  if (!waitUntilGazeboRenderingReady()) {
    progress.close();
    if (launch_proc_) {
      qt::qErrorBox(this, "Failed to get the Gazebo rendering information.");
      reset();
    }
    return false;
  }
  progress.progressStep();

  // Start dynamic parameters.
  progress.setLabelText("Starting dynamic configuration.");
  if (!dynamic_config_->start(kWaitForServerTimeout)) {
    progress.close();
    if (launch_proc_) {
      qt::qErrorBox(this, "Failed to start the dynamic configuration manager.");
      reset();
    }
    return false;
  }
  progress.progressStep();

  // Start commanders.
  progress.setLabelText("Starting commanders.");
  if (!commanders_->start(kWaitForServerTimeout)) {
    progress.close();
    if (launch_proc_) {
      qt::qErrorBox(this, "Failed to start the commanders.");
      reset();
    }
    return false;
  }
  progress.progressStep();

  progress.close();
  return true;
}

void SimulationWidget::terminateSITL()
{
  Q_EMIT telemetryLossExpected();

  // Stop the Gazebo process on another thread.
  qInfo() << "Terminating Gazebo";
  spinner_.start();
  terminateSimulationAndWait();
  spinner_.stop();

  qInfo() << "Restoring to the state before the simulation started.";
  reset();

  Q_EMIT terminated();
  qt::qInfoBox(this, "SITL has been terminated successfully.");
}

bool SimulationWidget::startHITL()
{
  // Confirm that the vehicle is not armed.
  if (!arming_) {
    qt::qWarnBox(this, "This operation cannot be performed because the arming status has not been received yet.");
    return false;
  }
  else {
    if (arming_->data) {
      qt::qWarnBox(this, "This operation cannot be performed while the vehicle is armed.");
      return false;
    }
  }

  // Create a progress bar.
  qt::ProgressDialog progress("Start HITL", 10, this);
  progress.setCancelButton(nullptr);
  progress.show();

  // Build local packages.
  progress.setLabelText("Building the Tobas project.");
  const auto local_build_res = cmn::buildLocalProject(proj_paths_.getProjPath());
  if (!local_build_res) {
    qt::qErrorBox(this, "Failed to build the Tobas project:\n\n" + local_build_res.error());
    progress.close();
    return false;
  }
  progress.progressStep();

  // Connect over SSH.
  progress.setLabelText("Connecting to the flight controller.");
  if (ssh_client_->connect() != ssh::SshClient::kNoError) {
    qt::qErrorBox(this, "No SSH connection: " + QString(ssh_client_->errorMessage()));
    progress.close();
    return false;
  }
  progress.progressStep();

  // Stop the Real service.
  progress.setLabelText("Stopping the Tobas real service.");
  Q_EMIT telemetryLossExpected();
  if (ssh_client_->execute("systemctl stop tobas_real.target", true) != ssh::SshClient::kNoError) {
    qt::qErrorBox(this, "Failed to stop Tobas real service:\n\n" + QString(ssh_client_->errorMessage()));
    progress.close();
    return false;
  }
  progress.progressStep();

  // Send Tobas packages.
  progress.setLabelText("Sending the Tobas project to the flight controller.");
  const auto& proj_path = proj_paths_.getProjPath();
  const auto mesh_path = proj_paths_.cfgMeshDirPath();
  const auto remote_dir = fs::path(kColconWSPathRoot) / "src/";
  if (ssh_client_->scpPut(proj_path, remote_dir, true, { mesh_path }, true) != ssh::SshClient::kNoError) {
    qt::qErrorBox(this, "Failed to send Tobas project:\n\n" + QString(ssh_client_->errorMessage()));
    progress.close();
    return false;
  }
  progress.progressStep();

  // Build remote packages.
  progress.setLabelText("Building the Tobas project.");
  if (!remote_proj_builder_->build(proj_paths_.remoteProjPath())) {
    qt::qErrorBox(this, "Failed to build the Tobas project:\n\n" + QString(remote_proj_builder_->getErrorMessage()));
    progress.close();
    return false;
  }
  progress.progressStep();

  // Wait for the Gazebo server to start.
  progress.setLabelText("Launching the simulation.");
  launchSimulation(true);  // FIXME: Improve communication load by starting core on the RPi side.
  if (!waitUntilGazeboServerReady()) {
    qt::qErrorBox(this, "Failed to start the Gazebo server.");
    progress.close();
    return false;
  }
  progress.progressStep();

  // Wait for Gazebo rendering to start.
  progress.setLabelText("Waiting for Gazebo rendering to start.");
  if (!waitUntilGazeboRenderingReady()) {
    qt::qErrorBox(this, "Failed to get the Gazebo rendering information.");
    progress.close();
    return false;
  }
  progress.progressStep();

  // Start the HITL service.
  progress.setLabelText("Starting the Tobas HITL service.");
  if (ssh_client_->execute("systemctl restart tobas_hitl.service", true) != ssh::SshClient::kNoError) {
    qt::qErrorBox(this, "Failed to restart Tobas HITL service:\n\n" + QString(ssh_client_->errorMessage()));
    progress.close();
    return false;
  }
  progress.progressStep();

  // Start dynamic parameters.
  progress.setLabelText("Starting dynamic configuration.");
  if (!dynamic_config_->start(kWaitForServerTimeout)) {
    progress.close();
    return false;
  }
  progress.progressStep();

  // Start commanders.
  progress.setLabelText("Starting commanders.");
  if (!commanders_->start(kWaitForServerTimeout)) {
    progress.close();
    return false;
  }
  progress.progressStep();

  progress.close();
  return true;
}

void SimulationWidget::terminateHITL()
{
  Q_EMIT telemetryLossExpected();

  // Create a progress bar.
  qt::ProgressDialog progress("Terminate HITL", 3, this);
  progress.setCancelButton(nullptr);
  progress.show();

  // Stop the launch process.
  progress.setLabelText("Terminating the simulation.");
  terminateSimulationAndWait();
  progress.progressStep();

  // Stop the HITL service.
  progress.setLabelText("Stopping the Tobas HITL service.");
  if (ssh_client_->execute("systemctl stop tobas_hitl.service", true) != ssh::SshClient::kNoError) {
    qt::qErrorBox(this, "Failed to stop Tobas HITL service:\n\n" + QString(ssh_client_->errorMessage()));
    progress.close();
    reset();
    return;
  }
  progress.progressStep();

  // Start the Real service.
  progress.setLabelText("Starting the Tobas real service.");
  if (ssh_client_->execute("systemctl restart tobas_real.target", true) != ssh::SshClient::kNoError) {
    qt::qErrorBox(this, "Failed to start Tobas real service:\n\n" + QString(ssh_client_->errorMessage()));
    progress.close();
    reset();
    return;
  }
  progress.progressStep();

  progress.close();

  reset();
  Q_EMIT terminated();
  qt::qInfoBox(this, "HITL has been terminated successfully.");
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
    launch_proc_ = nullptr;
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
  launch_proc_ = nullptr;
}

void SimulationWidget::terminateSimulation()
{
  terminateLaunchProcess();

  if (!sim::killGazeboServer()) {
    throw std::runtime_error("Failed to kill Gazebo server.");
  }
}

void SimulationWidget::terminateSimulationAndWait()
{
  terminateLaunchProcess();

  qInfo() << "Waiting for Gazebo to shut down.";
  if (!sim::killGazeboServerAndWait(node_)) {
    throw std::runtime_error("Failed to kill Gazebo server.");
  }
}

void SimulationWidget::onStartRequested()
{
  qDebug() << "SimulationWidget::onStartRequested";

  bool success;
  switch (sim_settings_->loopType()) {
    case LoopType::SITL:
      success = startSITL();
      break;
    case LoopType::HITL:
      success = startHITL();
      break;
    default:
      throw;
  }

  if (!success) {
    reset();
    return;
  }

  sim_settings_->setEnabled(false);
  dynamic_config_->setEnabled(true);
  commanders_->setEnabled(true);

  qt::qInfoBox(this, "The simulation has started successfully.");

  Q_EMIT started();
}

void SimulationWidget::onTerminateRequested()
{
  qDebug() << "SimulationWidget::onTerminateRequested";

  switch (sim_settings_->loopType()) {
    case LoopType::SITL:
      terminateSITL();
      break;
    case LoopType::HITL:
      terminateHITL();
      break;
    default:
      throw;
  }
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
    launch_proc_ = nullptr;

    // Kill any Gazebo server that may still remain.
    if (!sim::killGazeboServer()) {
      throw std::runtime_error("Failed to kill Gazebo server.");
    }

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

void SimulationWidget::armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming)
{
  arming_ = arming;
}
}  // namespace sim
}  // namespace gui
}  // namespace tobas
