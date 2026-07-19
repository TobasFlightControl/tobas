// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_gcs/gcs.hpp"

#include <QButtonGroup>
#include <QHBoxLayout>
#include <QVBoxLayout>

#include <tobas_constants/path.hpp>
#include <tobas_cyclonedds_config/cyclonedds_config.hpp>
#include <tobas_gui_common/constants.hpp>
#include <tobas_gui_common/load_project_dialog.hpp>
#include <tobas_gui_common/remote_project_builder.hpp>
#include <tobas_qt_tools/event.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/path.hpp>
#include <tobas_qt_tools/thread.hpp>
#include <tobas_qt_tools/util.hpp>
#include <tobas_qt_tools/widgets/progress_dialog.hpp>
#include <tobas_qt_tools/widgets/stacked_widget.hpp>
#include <tobas_ros2_tools/util.hpp>
#include <tobas_string_tools/stream.hpp>

#include "tobas_gcs/app_button.hpp"
#include "tobas_gcs/util.hpp"

using namespace std::chrono_literals;
namespace fs = std::filesystem;

namespace tobas
{
namespace gui
{
namespace gcs
{
GroundControlStationWidget::GroundControlStationWidget(rclcpp::Node::SharedPtr node)
  : bridge_(node)
  , network_checker_(this, bridge_)
  , property_client_(node, "tobas_gcs/gcs")
  , ssh_client_(node)
  , remote_proj_builder_(node)
  , spinner_(Qt::WindowModal, this)
{
  // Applications
  sensor_calib_ = new sc::SensorCalibrationWidget(node, bridge_, drone_);
  actuator_test_ = new at::ActuatorTestWidget(node, bridge_, tree_, drone_);
  control_system_ = new ctrl::ControlSystemWidget(node, bridge_, drone_);
  param_tuning_ = new param::ParameterTuningWidget(node);
  flight_log_ = new log::FlightLogWidget(node, bridge_);
  simulation_ = new sim::SimulationWidget(node, bridge_);

  const auto rsrc_dir = QString::fromStdString(getResourceDir() / "tool");
  const auto sensor_calib_btn = new AppButton("Sensor Calib", rsrc_dir + "/sensor_calibration.svg");
  const auto actuator_test_btn = new AppButton("Actuator Test", rsrc_dir + "/actuator_test.svg");
  const auto control_system_btn = new AppButton("Control System", rsrc_dir + "/control_system.svg");
  const auto param_tuning_btn = new AppButton("Param Tuning", rsrc_dir + "/parameter_tuning.svg");
  const auto flight_log_btn = new AppButton("Flight Log", rsrc_dir + "/flight_log.svg");
  const auto simulation_btn = new AppButton("Simulation", rsrc_dir + "/simulation.svg");

  const auto app_sw = new qt::StackedWidget();
  app_sw->addWidget(sensor_calib_);
  app_sw->addWidget(actuator_test_);
  app_sw->addWidget(control_system_);
  app_sw->addWidget(param_tuning_);
  app_sw->addWidget(flight_log_);
  app_sw->addWidget(simulation_);

  const auto btn_group = new QButtonGroup(this);
  int btn_id = 0;
  btn_group->addButton(sensor_calib_btn, btn_id++);
  btn_group->addButton(actuator_test_btn, btn_id++);
  btn_group->addButton(control_system_btn, btn_id++);
  btn_group->addButton(param_tuning_btn, btn_id++);
  btn_group->addButton(flight_log_btn, btn_id++);
  btn_group->addButton(simulation_btn, btn_id++);
  btn_group->buttons().first()->setChecked(true);

  // Connection checker
  remote_conn_ = new RemoteConnectionWidget(bridge_);
  remote_conn_->setMaximumHeight(sensor_calib_btn->height());

  // Package manager
  proj_path_ = new QLineEdit();
  proj_path_->setMaximumWidth(kPathMaxWidth);
  proj_path_->setReadOnly(true);
  proj_path_->setFocusPolicy(Qt::NoFocus);

  load_btn_ = new QPushButton("Load Project");
  write_btn_ = new QPushButton("Write Project");
  write_btn_->setEnabled(false);

  // Power control buttons
  restart_btn_ = new RestartButton(kPowerButtonRadius);
  shutdown_btn_ = new ShutdownButton(kPowerButtonRadius);
  restart_btn_->setEnabled(false);
  shutdown_btn_->setEnabled(false);

  // Layout
  const auto pkg_btn_cols = new QHBoxLayout();
  pkg_btn_cols->addWidget(load_btn_);
  pkg_btn_cols->addWidget(write_btn_);

  const auto pkg_rows = new QVBoxLayout();
  pkg_rows->addWidget(proj_path_);
  pkg_rows->addLayout(pkg_btn_cols);

  const auto header_cols = new QHBoxLayout();
  header_cols->addWidget(sensor_calib_btn, 1);
  header_cols->addWidget(actuator_test_btn, 1);
  header_cols->addWidget(control_system_btn, 1);
  header_cols->addWidget(param_tuning_btn, 1);
  header_cols->addWidget(flight_log_btn, 1);
  header_cols->addWidget(simulation_btn, 1);
  header_cols->addStretch();
  header_cols->addWidget(remote_conn_);
  header_cols->addLayout(pkg_rows);
  qt::addSpacing(header_cols, 30, QSizePolicy::Preferred);  // Collapse this when there is not enough space.
  header_cols->addWidget(restart_btn_);
  header_cols->addWidget(shutdown_btn_);

  const auto rows = new QVBoxLayout();
  rows->addLayout(header_cols);
  rows->addWidget(app_sw);

  setLayout(rows);

  // Connection
  connect(btn_group, &QButtonGroup::idClicked, app_sw, &QStackedWidget::setCurrentIndex);
  connect(load_btn_, &QPushButton::clicked, this, &self::onLoadButtonClicked);
  connect(write_btn_, &QPushButton::clicked, this, &self::onWriteButtonClicked);
  connect(restart_btn_, &QPushButton::clicked, this, &self::onRestartButtonClicked);
  connect(shutdown_btn_, &QPushButton::clicked, this, &self::onShutdownButtonClicked);
  connect(simulation_, &sim::SimulationWidget::started, this, &self::onSimRealStateChanged);
  connect(simulation_, &sim::SimulationWidget::terminated, this, &self::onSimRealStateChanged);
  connect(simulation_, &sim::SimulationWidget::telemetryLossExpected, this, &self::expectTelemetryLoss);
  connect(remote_conn_, &RemoteConnectionWidget::disconnected, this, &self::onRemoteConnectionDisconnected);
  connect(&bridge_, &RosQtBridge::armingReceived, this, &self::armingCb, Qt::QueuedConnection);
}

void GroundControlStationWidget::reset(bool include_simulation)
{
  qt::processAllQueuedEvents();

  clearExpectedTelemetryLoss();
  remote_conn_->restart();

  sensor_calib_->reset();
  actuator_test_->reset();
  control_system_->reset();
  param_tuning_->reset();
  flight_log_->reset();

  if (include_simulation) {
    simulation_->reset();
  }

  arming_.reset();

  qt::processAllQueuedEvents();
}

void GroundControlStationWidget::updateInternalDataStructures()
{
  const auto ns = '/' + drone_.name;
  const auto proj_path = projectPath();

  // First switch the topics and drain all callbacks from the previous vehicle.
  bridge_.initializeScopedTopics(ns);
  qt::processAllQueuedEvents();

  // Pass the connection information to the SSH interface.
  if (ssh_client_.setEndpoint(ssh_config_.host, ssh_config_.user) != ssh::SshClient::kNoError) {
    qt::qErrorBox(this, "Failed to set SSH configuration:\n" + QString(ssh_client_.errorMessage()));
    return;
  }

  reset();

  sensor_calib_->updateInternalDataStructures();
  actuator_test_->updateProject(proj_path);
  control_system_->updateInternalDataStructures();
  param_tuning_->updateProject(proj_path);
  flight_log_->updateNamespace(ns);
  simulation_->updateProject(proj_path);
}

void GroundControlStationWidget::closeEvent(QCloseEvent* event)
{
  qDebug() << "GroundControlStationWidget::closeEvent";

  sensor_calib_->close();
  actuator_test_->close();
  control_system_->close();
  param_tuning_->close();
  flight_log_->close();
  simulation_->close();

  event->accept();
}

fs::path GroundControlStationWidget::projectPath() const
{
  return proj_path_->text().toStdString();
}

void GroundControlStationWidget::expectTelemetryLoss()
{
  telemetry_loss_expected_ = true;
}

void GroundControlStationWidget::clearExpectedTelemetryLoss()
{
  telemetry_loss_expected_ = false;
}

std::expected<void, QString> GroundControlStationWidget::restartInBackground()
{
  // Run the command.
  const auto res = ssh_client_.execute("systemctl restart tobas_real.target", true);
  if (res != ssh::SshClient::kNoError) {
    return std::unexpected("Failed to restart the flight controller:\n\n" + QString(ssh_client_.errorMessage()));
  }

  return {};
}

std::expected<void, QString> GroundControlStationWidget::shutdownInBackground()
{
  // Run the command.
  const auto res = ssh_client_.execute("poweroff", true, true);
  if (res != ssh::SshClient::kNoError) {
    return std::unexpected("Failed to shutdown the flight controller:\n\n" + QString(ssh_client_.errorMessage()));
  }

  // Wait long enough for the Raspberry Pi to shut down reliably.
  qt::spinFor(5s);

  return {};
}

void GroundControlStationWidget::onLoadButtonClicked()
{
  qDebug() << "GroundControlStationWidget::onLoadButtonClicked";

  // Confirm that the simulation is not running.
  if (simulation_->isRunning()) {
    qt::qWarnBox(this, "Stop the simulation before loading a new project.");
    return;
  }

  // Get the previously opened path.
  std::string last_opened_dir;
  if (property_client_.get(kLastOpenedDirKey, last_opened_dir) < 0) {
    qWarning() << property_client_.errorMessage();
    last_opened_dir = ros2::expandUser(kColconWSPathHome) / "src";
    if (!fs::is_directory(last_opened_dir)) {
      last_opened_dir = ros2::getHomeDir();
    }
  }

  // Update the project path.
  cmn::LoadProjectDialog dialog(this, QString::fromStdString(last_opened_dir));
  if (dialog.exec() != QDialog::Accepted) {
    return;
  }
  const fs::path proj_path = dialog.selectedFiles().first().toStdString();
  proj_paths_.setProjPath(proj_path);

  // Check the version.
  if (proj_version_.load(proj_paths_.versionPath())) {
    if (!proj_version_.isCompatible()) {
      qt::qWarnBox(
        this,
        "The current Tobas version (" + cmn::Version::Current().toString() +
          ") is incompatible with the version used to create this project (" + proj_version_.toString() +
          "). Please update the project using the Setup Assistant.");
      return;
    }
  }
  else {
    qt::qWarnBox(this, "Failed to read the project version. Please update the project using the Setup Assistant.");
    return;
  }

  // Set the path text.
  proj_path_->setText(QString::fromStdString(proj_path));

  // Save the directory opened by the user.
  const auto par_dir = fs::path(proj_path).parent_path();
  if (property_client_.set(kLastOpenedDirKey, par_dir) < 0) {
    qWarning() << property_client_.errorMessage();
  }
  if (property_client_.save() < 0) {
    qWarning() << property_client_.errorMessage();
  }

  // Confirm that the vehicle configuration file exists.
  const auto tbsdrn_path = proj_paths_.tbsdrnPath();
  if (!fs::is_regular_file(tbsdrn_path)) {
    qt::qErrorBox(
      this, "\"" + QString::fromStdString(tbsdrn_path) + "\" does not exist. Please create a new Tobas project.");
    return;
  }

  // Load KDL tree.
  const auto uadf_path = proj_paths_.originalUadfPath();
  if (!uadf_parser_.parseFromPath(uadf_path, uadf_)) {
    qt::qErrorBox(this, "Failed to parse UADF:\n\n" + QString::fromStdString(uadf_parser_.errorMessage()));
    return;
  }
  if (!tree_parser_.parseFromUrdf(*uadf_.urdf, tree_)) {
    qt::qErrorBox(
      this, "Failed to construct KDL tree from URDF:\n\n" + QString::fromStdString(tree_parser_.errorMessage()));
    return;
  }

  // Load drone configuration.
  if (!drone_.load(tbsdrn_path)) {
    qt::qErrorBox(this, "Failed to load drone configuration.");
    return;
  }

  // Load SSH configuration.
  if (!ssh_config_.load(proj_paths_.sshConfigPath())) {
    qt::qErrorBox(this, "Failed to load SSH configuration.");
    return;
  }

  // Load network configuration.
  if (!network_config_.load(proj_paths_.networkConfigPath())) {
    qt::qErrorBox(this, "Failed to load network configuration.");
    return;
  }

  // Update the internal state.
  updateInternalDataStructures();

  // Enable control buttons.
  write_btn_->setEnabled(true);
  restart_btn_->setEnabled(true);
  shutdown_btn_->setEnabled(true);

  // Show a dialog indicating that the project was loaded successfully.
  qt::qInfoBox(this, "Tobas project is loaded successfully.");
}

void GroundControlStationWidget::onWriteButtonClicked()
{
  qDebug() << "GroundControlStationWidget::onWriteButtonClicked";

  // Confirm that the vehicle is not armed.
  if (!arming_) {
    if (!qt::yesOrNo(
          this,
          "This operation will restart the flight control software, "
          "so it can only be performed when the aircraft is completely stationary. "
          "Do you want to proceed?",
          qt::WARN)) {
      return;
    }
  }
  else {
    if (arming_->data) {
      qt::qWarnBox(this, "This operation cannot be performed while the vehicle is armed.");
      return;
    }
  }

  const auto proj_path = projectPath();
  const auto remote_proj_path = proj_paths_.remoteProjPath();
  const auto config_pkg_name = proj_paths_.cfgPkgName();

  // Create a progress bar.
  qt::ProgressDialog progress("Write Tobas Project", 12, this);
  progress.setCancelButton(nullptr);
  progress.show();

  // Connect over SSH.
  progress.setLabelText("Connecting to the flight controller.");
  if (ssh_client_.connect() != ssh::SshClient::kNoError) {
    progress.close();
    qt::qErrorBox(this, "No SSH connection: " + QString(ssh_client_.errorMessage()));
    return;
  }
  progress.progressStep();

  // Check the FC version.
  progress.setLabelText("Checking the Tobas version.");
  std::string fc_ver_text;
  if (ssh_client_.execute("/opt/tobas/lib/tobas_version/show_version", fc_ver_text) != ssh::SshClient::kNoError) {
    progress.close();
    qt::qErrorBox(this, "Failed to retrieve the firmware version: " + QString(ssh_client_.errorMessage()));
    return;
  }
  cmn::Version fc_version;
  if (!fc_version.fromString(QString::fromStdString(fc_ver_text))) {
    progress.close();
    qt::qErrorBox(this, "Failed to parse the firmware version: " + QString::fromStdString(fc_ver_text));
    return;
  }
  if (!fc_version.isCompatible(proj_version_)) {
    progress.close();
    qt::qWarnBox(
      this,
      "The firmware version (" + fc_version.toString() +
        ") is incompatible with the version used to create this project (" + proj_version_.toString() + ").");
    return;
  }
  progress.progressStep();

  // Stop the service.
  progress.setLabelText("Stopping the Tobas real service.");
  expectTelemetryLoss();
  if (ssh_client_.execute("systemctl stop tobas_real.target", true) != ssh::SshClient::kNoError) {
    clearExpectedTelemetryLoss();
    progress.close();
    qt::qErrorBox(this, "Failed to stop Tobas real service:\n\n" + QString(ssh_client_.errorMessage()));
    return;
  }
  progress.progressStep();

  // Load environment variables; continue even if they cannot be loaded.
  progress.setLabelText("Getting environment variables.");
  std::string project_env_text;
  if (ssh_client_.sftpRead(kProjectEnvPath, project_env_text, true) == ssh::SshClient::kNoError) {
    if (!project_env_parser_.parseFromText(project_env_text)) {
      progress.close();
      qt::qErrorBox(this, "Failed to parse configuration file.");
      return;
    }
  }
  else {
    qWarning() << "Failed to get the current environment variables: " << ssh_client_.errorMessage();
  }
  progress.progressStep();

  // Use a clean build when packages change to avoid conflicts.
  if (config_pkg_name != project_env_parser_.config_pkg) {
    // Initialize the workspace.
    progress.setLabelText("Initializing colcon workspace.");
    if (ssh_client_.execute(std::format("rm -rf {}", kColconWSPathRoot), true)) {
      progress.close();
      qt::qErrorBox(this, "Failed to remove the old colcon workspace:\n\n" + QString(ssh_client_.errorMessage()));
      return;
    }
    if (ssh_client_.execute(std::format("mkdir -p {}/src", kColconWSPathRoot), true)) {
      progress.close();
      qt::qErrorBox(this, "Failed to create a new colcon workspace:\n\n" + QString(ssh_client_.errorMessage()));
      return;
    }
    progress.progressStep();
  }
  else {
    progress.progressStep();
  }

  // Update environment variables.
  progress.setLabelText("Setting environment variables.");
  project_env_parser_.config_pkg = config_pkg_name;
  project_env_parser_.nif = network_config_.interface;
  if (ssh_client_.sftpWrite(kProjectEnvPath, project_env_parser_.exportText(), true) != ssh::SshClient::kNoError) {
    progress.close();
    qt::qErrorBox(this, "Failed to set environment variables:\n\n" + QString(ssh_client_.errorMessage()));
    return;
  }
  progress.progressStep();

  // Send the project.
  progress.setLabelText("Sending the Tobas project to the flight controller.");
  const auto remote_dir = fs::path(kColconWSPathRoot) / "src/";
  const auto mesh_path = proj_paths_.cfgMeshDirPath();
  const auto git_path = proj_paths_.getProjPath() / ".git";
  if (ssh_client_.scpPut(proj_path, remote_dir, true, { mesh_path, git_path }, true) != ssh::SshClient::kNoError) {
    progress.close();
    qt::qErrorBox(this, "Failed to send Tobas project:\n\n" + QString(ssh_client_.errorMessage()));
    return;
  }
  progress.progressStep();

  // Build the project on another thread so the GUI does not stop.
  progress.setLabelText("Building the Tobas project.");
  if (!remote_proj_builder_.build(proj_paths_.remoteProjPath())) {
    const QString error_msg(remote_proj_builder_.getErrorMessage());
    if (error_msg.size() < cmn::kSaveLogTextSizeThresh) {
      qt::qErrorBox(this, "Failed to build the Tobas project:\n\n" + error_msg);
    }
    else {
      const auto log_path =
        qt::writeTimestampedFile(error_msg + '\n', qt::expandUser(kGuiLogDir), "", "builderr_project_remote");
      if (log_path) {
        qt::qErrorBox(this, "Failed to build the Tobas project. The output has been saved to:\n" + log_path.value());
      }
      else {
        qt::qErrorBox(this, "Failed to build the Tobas project, and also failed to save the error message.");
      }
    }
    progress.close();
    return;
  }
  progress.progressStep();

  // Update the DDS settings.
  progress.setLabelText("Writing DDS configuration.");
  cyclonedds::Data dds_data;
  dds_data.interfaces.emplace_back("lo", 1, true);  // Multicast must be enabled to bridge the two NICs.
  dds_data.interfaces.emplace_back(network_config_.interface, 0, true);
  const auto dds_config_if_text = cyclonedds::exportText(dds_data);
  if (ssh_client_.sftpWrite(kCycloneddsConfigPath, dds_config_if_text, true) != ssh::SshClient::kNoError) {
    progress.close();
    qt::qErrorBox(this, "Failed to write DDS configuration:\n\n" + QString(ssh_client_.errorMessage()));
    return;
  }
  progress.progressStep();

  // Enable the service.
  progress.setLabelText("Enabling the flight controller.");
  if (ssh_client_.execute("systemctl enable tobas_real.target", true) != ssh::SshClient::kNoError) {
    progress.close();
    qt::qErrorBox(this, "Failed to enable Tobas real service:\n\n" + QString(ssh_client_.errorMessage()));
    return;
  }
  progress.progressStep();

  // Start the service.
  progress.setLabelText("Starting the flight controller.");
  if (ssh_client_.execute("systemctl start tobas_real.target", true) != ssh::SshClient::kNoError) {
    progress.close();
    qt::qErrorBox(this, "Failed to start Tobas real service:\n\n" + QString(ssh_client_.errorMessage()));
    return;
  }
  progress.progressStep();

  // Reload.
  progress.setLabelText("Reloading.");
  reset();
  progress.progressStep();

  progress.close();
  qt::qInfoBox(this, "Tobas project is installed successfully.");
}

void GroundControlStationWidget::onRestartButtonClicked(bool checked)
{
  qDebug() << "GroundControlStationWidget::onRestartButtonClicked";

  if (!checked) {
    return;
  }

  // Confirm that the vehicle is not armed.
  if (arming_ && arming_->data) {
    qt::qWarnBox(this, "This operation cannot be performed while the vehicle is armed.");
    restart_btn_->setChecked(false);
    return;
  }

  // Confirm before restarting.
  if (!qt::yesOrNo(this, "Are you sure you want to restart the flight controller?", qt::WARN)) {
    restart_btn_->setChecked(false);
    return;
  }

  // Restart the systemd service.
  expectTelemetryLoss();
  spinner_.start();
  const auto res = restartInBackground();
  spinner_.stop();

  if (res) {
    qt::qInfoBox(this, "The flight controller has been restarted successfully.");
    reset();
  }
  else {
    clearExpectedTelemetryLoss();
    qt::qErrorBox(this, res.error());
  }

  // Return the button to the state before it was pressed.
  restart_btn_->setChecked(false);
}

void GroundControlStationWidget::onShutdownButtonClicked(bool checked)
{
  qDebug() << "GroundControlStationWidget::onShutdownButtonClicked";

  if (!checked) {
    return;
  }

  // Confirm that the vehicle is not armed.
  if (arming_ && arming_->data) {
    qt::qWarnBox(this, "This operation cannot be performed while the vehicle is armed.");
    shutdown_btn_->setChecked(false);
    return;
  }

  // Confirm before shutting down.
  if (!qt::yesOrNo(this, "Are you sure you want to shut down the FC?", qt::WARN)) {
    shutdown_btn_->setChecked(false);
    return;
  }

  // Shut down the OS.
  expectTelemetryLoss();
  spinner_.start();
  const auto res = shutdownInBackground();
  spinner_.stop();

  if (res) {
    qt::qInfoBox(this, "The flight controller has been shut down successfully.");
    reset();  // Call `reset()` last so ROS messages and other state held by widgets are reliably reset.
  }
  else {
    clearExpectedTelemetryLoss();
    qt::qErrorBox(this, res.error());
  }

  // Return the button to the state before it was pressed.
  shutdown_btn_->setChecked(false);
}

void GroundControlStationWidget::onSimRealStateChanged()
{
  qDebug() << "GroundControlStationWidget::onSimRealStateChanged";

  // Reset everything except the simulation widget.
  reset(false);
}

void GroundControlStationWidget::onRemoteConnectionDisconnected()
{
  qDebug() << "GroundControlStationWidget::onRemoteConnectionDisconnected";

  if (!telemetry_loss_expected_) {
    qt::qWarnBox(this, "Telemetry reception timed out. The connection to the flight controller may have been lost.");
  }

  reset();
}

void GroundControlStationWidget::armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming)
{
  arming_ = arming;
}
}  // namespace gcs
}  // namespace gui
}  // namespace tobas
