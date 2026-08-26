// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_gcs/gcs.hpp"

#include <exception>
#include <limits>
#include <utility>

#include <QButtonGroup>
#include <QHBoxLayout>
#include <QLabel>
#include <QSignalBlocker>
#include <QVBoxLayout>

#include <tobas_constants/path.hpp>
#include <tobas_cyclonedds_config/cyclonedds_config.hpp>
#include <tobas_gui_common/constants.hpp>
#include <tobas_gui_common/load_project_dialog.hpp>
#include <tobas_gui_common/project_paths.hpp>
#include <tobas_gui_common/remote_project_builder.hpp>
#include <tobas_qt_tools/event.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/path.hpp>
#include <tobas_qt_tools/thread.hpp>
#include <tobas_qt_tools/util.hpp>
#include <tobas_qt_tools/widgets/progress_dialog.hpp>
#include <tobas_qt_tools/widgets/stacked_widget.hpp>
#include <tobas_rqt_bridge/wait_for_message.hpp>
#include <tobas_std_tools/check.hpp>

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
namespace
{
constexpr auto kHostRole = Qt::UserRole;
constexpr int kHeartbeatTimeout = 10000;  // [ms]
constexpr char kIdPrefix[] = "id";

std::unique_ptr<ros2::AsyncNodeManager> createRosNodeManager(
  const QString& static_peer,
  std::vector<std::string> ros_args,
  const rclcpp::Context::SharedPtr& context)
{
  TOBAS_CHECK(qputenv("ROS_STATIC_PEERS", static_peer.toUtf8()));

  std::vector<char*> ros_argv;
  for (auto& arg : ros_args) {
    ros_argv.push_back(arg.data());
  }

  rclcpp::InitOptions context_options;
  context_options.shutdown_on_signal = false;
  context_options.auto_initialize_logging(false);
  context->init(static_cast<int>(ros_argv.size()), ros_argv.data(), context_options);

  return std::make_unique<ros2::AsyncNodeManager>(context, "tobas_gcs");
}
}  // namespace

GroundControlStationWidget::GroundControlStationWidget(int argc, char** argv) : spinner_(Qt::WindowModal, this)
{
  for (int i = 0; i < argc; ++i) {
    ros_args_.emplace_back(argv[i]);
  }

  // Applications
  sensor_calib_ = new sc::SensorCalibrationWidget(bridge_, drone_);
  actuator_test_ = new at::ActuatorTestWidget(bridge_, tree_, drone_);
  control_system_ = new ctrl::ControlSystemWidget(bridge_, drone_);
  param_tuning_ = new param::ParameterTuningWidget();
  flight_log_ = new log::FlightLogWidget(bridge_);
  simulation_ = new sim::SimulationWidget(bridge_);

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

  // Default page
  app_sw->setCurrentWidget(control_system_);
  control_system_btn->setChecked(true);

  // Connection checker
  remote_conn_ = new RemoteConnectionWidget(bridge_);
  remote_conn_->setMaximumHeight(sensor_calib_btn->height());

  // Package manager
  proj_path_ = new QLineEdit();
  proj_path_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  proj_path_->setMaximumWidth(400);
  proj_path_->setReadOnly(true);
  proj_path_->setFocusPolicy(Qt::NoFocus);
  load_btn_ = new QPushButton("Load");

  // FC selection
  fc_scanner_ = new FlightControllerScanner(this);
  fc_selector_ = new QComboBox();
  fc_selector_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  vehicle_id_ = new QSpinBox();
  vehicle_id_->setRange(0, std::numeric_limits<int>::max());
  vehicle_id_->setValue(0);
  connect_btn_ = new qt::ToggleButton("Connect", "Disconnect");

  // Control buttons
  write_btn_ = new QPushButton("Write");
  restart_btn_ = new QPushButton("Restart");
  shutdown_btn_ = new QPushButton("Shutdown");
  write_btn_->setEnabled(false);
  restart_btn_->setEnabled(false);
  shutdown_btn_->setEnabled(false);
  setFlightControllerPlaceholder("Searching for FCs...");

  // Layout
  const auto project_cols = new QHBoxLayout();
  project_cols->addWidget(proj_path_);
  project_cols->addWidget(load_btn_);

  const auto connection_cols = new QHBoxLayout();
  connection_cols->addWidget(new QLabel("Endpoint"));
  connection_cols->addWidget(fc_selector_);
  connection_cols->addWidget(new QLabel("ID"));
  connection_cols->addWidget(vehicle_id_);
  connection_cols->addWidget(connect_btn_);

  const auto configuration_rows = new QVBoxLayout();
  configuration_rows->addLayout(project_cols);
  configuration_rows->addLayout(connection_cols);

  const auto control_rows = new QVBoxLayout();
  control_rows->addWidget(write_btn_);
  control_rows->addWidget(restart_btn_);
  control_rows->addWidget(shutdown_btn_);

  const auto header_cols = new QHBoxLayout();
  header_cols->addWidget(sensor_calib_btn);
  header_cols->addWidget(actuator_test_btn);
  header_cols->addWidget(control_system_btn);
  header_cols->addWidget(param_tuning_btn);
  header_cols->addWidget(flight_log_btn);
  header_cols->addWidget(simulation_btn);
  header_cols->addStretch();
  header_cols->addLayout(configuration_rows);
  header_cols->addWidget(remote_conn_, 0, Qt::AlignVCenter);
  header_cols->addLayout(control_rows);

  const auto rows = new QVBoxLayout();
  rows->addLayout(header_cols);
  rows->addWidget(app_sw);

  setLayout(rows);

  // Connection
  connect(btn_group, &QButtonGroup::idClicked, app_sw, &QStackedWidget::setCurrentIndex);
  connect(fc_selector_, qOverload<int>(&QComboBox::currentIndexChanged), this, &self::updateHeaderActionAvailability);
  connect(vehicle_id_, &QSpinBox::textChanged, this, &self::updateHeaderActionAvailability);
  connect(load_btn_, &QPushButton::clicked, this, &self::onLoadButtonClicked);
  connect(connect_btn_, &qt::ToggleButton::checked, this, &self::onConnectRequested);
  connect(connect_btn_, &qt::ToggleButton::unchecked, this, &self::onDisconnectRequested);
  connect(write_btn_, &QPushButton::clicked, this, &self::onWriteButtonClicked);
  connect(fc_scanner_, &FlightControllerScanner::finished, this, &self::onFlightControllerScanFinished);
  connect(fc_scanner_, &FlightControllerScanner::failed, this, &self::onFlightControllerScanFailed);
  connect(restart_btn_, &QPushButton::clicked, this, &self::onRestartButtonClicked);
  connect(shutdown_btn_, &QPushButton::clicked, this, &self::onShutdownButtonClicked);
  connect(simulation_, &sim::SimulationWidget::started, this, &self::onSimRealStateChanged);
  connect(simulation_, &sim::SimulationWidget::terminated, this, &self::onSimRealStateChanged);
  connect(simulation_, &sim::SimulationWidget::telemetryLossExpected, this, &self::expectTelemetryLoss);
  connect(remote_conn_, &RemoteConnectionWidget::disconnected, this, &self::onRemoteConnectionDisconnected);
  connect(&bridge_, &RosQtBridge::armingReceived, this, &self::armingCb, Qt::QueuedConnection);

  updateHeaderActionAvailability();
}

void GroundControlStationWidget::closeEvent(QCloseEvent* event)
{
  qDebug() << "GroundControlStationWidget::closeEvent";

  clearRosConnection();

  sensor_calib_->close();
  actuator_test_->close();
  control_system_->close();
  param_tuning_->close();
  flight_log_->close();
  simulation_->close();

  event->accept();
}

void GroundControlStationWidget::reset(bool include_simulation)
{
  qt::processAllQueuedEvents();

  clearExpectedTelemetryLoss();
  if (connection_ready_) {
    remote_conn_->restart();
  }
  else {
    remote_conn_->stop();
  }

  sensor_calib_->reset();
  actuator_test_->reset();
  control_system_->reset();
  param_tuning_->reset();
  flight_log_->reset();

  if (include_simulation) {
    simulation_->reset();
  }

  arming_.reset();

  updateHeaderActionAvailability();

  qt::processAllQueuedEvents();
}

void GroundControlStationWidget::updateInternalDataStructures()
{
  const auto proj_path = proj_path_->text().toStdString();

  sensor_calib_->updateInternalDataStructures();
  actuator_test_->updateProject(proj_path);
  control_system_->updateInternalDataStructures();
  param_tuning_->updateProject(proj_path);
  flight_log_->onProjectLoaded();
  simulation_->updateProject(proj_path);
}

void GroundControlStationWidget::initializeRosConnection()
{
  const auto host = currentHost();
  const auto id = currentId();

  ros_context_ = std::make_shared<rclcpp::Context>();
  ros_node_manager_ = createRosNodeManager(host, ros_args_, ros_context_);
  ros_node_ = ros_node_manager_->node();

  ssh_client_.emplace(ros_node_);
  remote_proj_builder_.emplace(ros_node_);

  TOBAS_CHECK(ssh_client_->setEndpoint(host.toStdString(), cmn::kUserNameFC) == ssh::SshClient::kNoError);

  const auto ns = path::join("/", drone_.name, kIdPrefix + std::to_string(id));
  bridge_.initializeRosInterfaces(ros_node_, ns);
  sensor_calib_->initializeRosInterfaces(ros_node_, ns);
  actuator_test_->initializeRosInterfaces(ros_node_, ns);
  control_system_->initializeRosInterfaces(ros_node_, ns);
  param_tuning_->initializeRosInterfaces(ros_node_, ns);
  flight_log_->initializeRosInterfaces(ros_node_, ns);
  simulation_->initializeRosInterfaces(ros_node_, ns);

  connection_ready_ = true;
}

void GroundControlStationWidget::clearRosConnection()
{
  bridge_.clearRosInterfaces();
  sensor_calib_->clearRosInterfaces();
  actuator_test_->clearRosInterfaces();
  control_system_->clearRosInterfaces();
  param_tuning_->clearRosInterfaces();
  flight_log_->clearRosInterfaces();
  simulation_->clearRosInterfaces();

  ssh_client_.reset();
  remote_proj_builder_.reset();

  ros_node_.reset();
  if (ros_node_manager_) {
    ros_node_manager_->clear();
  }
  ros_node_manager_.reset();

  if (ros_context_ && ros_context_->is_valid()) {
    ros_context_->shutdown("Changing the GCS connection endpoint.");
  }
  ros_context_.reset();

  connection_ready_ = false;
}

void GroundControlStationWidget::connectToFlightController()
{
  TOBAS_CHECK(project_loaded_);

  fc_scanner_->stop();

  clearRosConnection();
  initializeRosConnection();
  reset();
}

void GroundControlStationWidget::disconnectFromFlightController()
{
  TOBAS_CHECK(project_loaded_);

  clearRosConnection();
  reset();

  fc_scanner_->start();
}

bool GroundControlStationWidget::waitForHeartbeat() const
{
  return static_cast<bool>(waitForMessage<&RosQtBridge::remoteHeartbeatReceived>(bridge_, kHeartbeatTimeout));
}

void GroundControlStationWidget::updateHeaderActionAvailability()
{
  const auto target_ready = project_loaded_ && !currentHost().isEmpty();
  const auto disconnected = !connection_ready_;

  load_btn_->setEnabled(disconnected);
  fc_selector_->setEnabled(disconnected && project_loaded_ && fc_selector_->count() > 1);
  vehicle_id_->setEnabled(disconnected && project_loaded_);
  connect_btn_->setEnabled(target_ready);
  write_btn_->setEnabled(target_ready);
  restart_btn_->setEnabled(connection_ready_);
  shutdown_btn_->setEnabled(connection_ready_);
}

void GroundControlStationWidget::setFlightControllerPlaceholder(const QString& text)
{
  const QSignalBlocker block(fc_selector_);

  fc_selector_->clear();
  fc_selector_->addItem(text);
  fc_selector_->setCurrentIndex(0);

  updateHeaderActionAvailability();
}

QString GroundControlStationWidget::currentHost() const
{
  return fc_selector_->currentData(kHostRole).toString();
}

int GroundControlStationWidget::currentId() const
{
  return vehicle_id_->value();
}

QString GroundControlStationWidget::currentConnectionDescription() const
{
  if (currentHost().isEmpty()) {
    return "unconfigured FC";
  }

  return currentHost() + " [ID: " + QString::number(currentId()) + ']';
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
  const auto res = ssh_client_->execute("systemctl restart tobas_real.target", true);
  if (res != ssh::SshClient::kNoError) {
    return std::unexpected("Failed to restart the flight controller:\n\n" + QString(ssh_client_->errorMessage()));
  }

  return {};
}

std::expected<void, QString> GroundControlStationWidget::shutdownInBackground()
{
  // Run the command.
  const auto res = ssh_client_->execute("poweroff", true, true);
  if (res != ssh::SshClient::kNoError) {
    return std::unexpected("Failed to shutdown the flight controller:\n\n" + QString(ssh_client_->errorMessage()));
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
  auto default_dir = qt::expandUser(kColconWSPathHome) + "/src";
  if (!QFileInfo(default_dir).isDir()) {
    default_dir = QDir::homePath();
  }
  const auto last_opened_dir = settings_store_.value(kLastOpenedDirKey, default_dir).toString();

  // Update the project path.
  cmn::LoadProjectDialog dialog(this, last_opened_dir);
  if (dialog.exec() != QDialog::Accepted) {
    return;
  }
  const fs::path proj_path = dialog.selectedFiles().first().toStdString();
  cmn::ProjectPaths proj_paths(proj_path);

  // Validate the project before replacing the current project or connection.
  const auto cur_version = cmn::Version::Current();
  cmn::Version next_proj_version;
  if (next_proj_version.load(proj_paths.versionPath())) {
    if (!next_proj_version.isCompatible(cur_version)) {
      qt::qWarnBox(
        this,
        "The current FC version (" + cur_version.toString() +
          ") is incompatible with the version used to create this project (" + next_proj_version.toString() + ").");
      return;
    }
  }
  else {
    qt::qWarnBox(this, "Failed to read the project version. Please create a new Tobas project.");
    return;
  }

  // Confirm that the vehicle configuration file exists.
  const auto tbsdrn_path = proj_paths.tbsdrnPath();
  if (!fs::is_regular_file(tbsdrn_path)) {
    qt::qErrorBox(this, "The drone configuration file does not exist. Please create a new Tobas project.");
    return;
  }

  // Load KDL tree.
  uadf::Model next_uadf;
  const auto uadf_path = proj_paths.originalUadfPath();
  if (!uadf_parser_.parseFromPath(uadf_path, next_uadf)) {
    qt::qErrorBox(this, "Failed to parse UADF:\n\n" + QString::fromStdString(uadf_parser_.errorMessage()));
    return;
  }
  kdl::Tree next_tree;
  if (!tree_parser_.parseFromUrdf(*next_uadf.urdf, next_tree)) {
    qt::qErrorBox(
      this, "Failed to construct KDL tree from URDF:\n\n" + QString::fromStdString(tree_parser_.errorMessage()));
    return;
  }

  // Load drone configuration.
  Drone next_drone;
  if (!next_drone.load(tbsdrn_path)) {
    qt::qErrorBox(this, "Failed to load drone configuration.");
    return;
  }

  // Load network configuration.
  cmn::NetworkConfig next_network_config;
  if (!next_network_config.load(proj_paths.networkConfigPath())) {
    qt::qErrorBox(this, "Failed to load network configuration.");
    return;
  }

  proj_version_ = std::move(next_proj_version);
  uadf_ = std::move(next_uadf);
  tree_ = std::move(next_tree);
  drone_ = std::move(next_drone);
  network_config_ = std::move(next_network_config);

  // Commit the path only after every project file has been validated.
  proj_path_->setText(QString::fromStdString(proj_path));
  const auto par_dir = fs::path(proj_path).parent_path();
  settings_store_.setValue(kLastOpenedDirKey, QString::fromStdString(par_dir.string()));

  // Update the internal states.
  clearRosConnection();
  updateInternalDataStructures();
  reset();

  project_loaded_ = true;
  updateHeaderActionAvailability();
  fc_scanner_->start();

  // Show a dialog indicating that the project was loaded successfully.
  qt::qInfoBox(this, "Tobas project has been loaded successfully.");
}

void GroundControlStationWidget::onConnectRequested()
{
  qDebug() << "GroundControlStationWidget::onConnectRequested";

  connectToFlightController();

  spinner_.start();
  const auto heartbeat_received = waitForHeartbeat();
  spinner_.stop();

  if (!heartbeat_received) {
    disconnectFromFlightController();
    connect_btn_->setChecked(false);
    qt::qErrorBox(this, "Timed out waiting for a heartbeat from " + currentHost() + ".");
    return;
  }

  qt::qInfoBox(this, "The connection to " + currentConnectionDescription() + " has been established successfully.");
}

void GroundControlStationWidget::onDisconnectRequested()
{
  qDebug() << "GroundControlStationWidget::onDisconnectRequested";

  const auto connection = currentConnectionDescription();
  disconnectFromFlightController();
  qt::qInfoBox(this, "The connection to " + connection + " has been closed.");
}

void GroundControlStationWidget::onWriteButtonClicked()
{
  qDebug() << "GroundControlStationWidget::onWriteButtonClicked";

  if (!qt::yesOrNo(
        this,
        "This operation will restart the flight control software, "
        "so it can only be performed when the aircraft is completely stationary. "
        "Do you want to write the project to " +
          currentConnectionDescription() + "?",
        qt::WARN)) {
    return;
  }

  const auto proj_path = proj_path_->text().toStdString();
  cmn::ProjectPaths proj_paths(proj_path);

  const auto remote_proj_path = proj_paths.remoteProjPath();
  const auto config_pkg_name = proj_paths.cfgPkgName();

  // Create a progress bar.
  qt::ProgressDialog progress("Write Tobas Project", 12, this);
  progress.setCancelButton(nullptr);
  progress.show();

  // Connect to the flight controller.
  progress.setLabelText("Connecting to the flight controller.");
  connectToFlightController();
  if (ssh_client_->connect() != ssh::SshClient::kNoError) {
    progress.close();
    disconnectFromFlightController();
    qt::qErrorBox(this, "No SSH connection: " + QString(ssh_client_->errorMessage()));
    return;
  }
  progress.progressStep();

  // Check the FC version.
  progress.setLabelText("Checking the FC version.");
  std::string fc_ver_text;
  if (ssh_client_->execute("/opt/tobas/lib/tobas_version/show_version", fc_ver_text) != ssh::SshClient::kNoError) {
    progress.close();
    disconnectFromFlightController();
    qt::qErrorBox(this, "Failed to retrieve the FC version: " + QString(ssh_client_->errorMessage()));
    return;
  }
  cmn::Version fc_version;
  if (!fc_version.fromString(QString::fromStdString(fc_ver_text))) {
    progress.close();
    disconnectFromFlightController();
    qt::qErrorBox(this, "Failed to parse the FC version: " + QString::fromStdString(fc_ver_text));
    return;
  }
  if (!fc_version.isCompatible(proj_version_)) {
    progress.close();
    disconnectFromFlightController();
    qt::qWarnBox(
      this,
      "The FC version (" + fc_version.toString() + ") is incompatible with the version used to create this project (" +
        proj_version_.toString() + ").");
    return;
  }
  else {
    const auto cur_version = cmn::Version::Current();
    if (fc_version < cur_version) {
      progress.close();
      disconnectFromFlightController();
      qt::qWarnBox(
        this,
        "The FC version (" + fc_version.toString() + ") is older than the GCS version (" + cur_version.toString() +
          "). Please update the FC image to incorporate bug fixes and other updates.");
      return;
    }
  }
  progress.progressStep();

  // Stop the service.
  progress.setLabelText("Stopping the Tobas real service.");
  expectTelemetryLoss();
  if (ssh_client_->execute("systemctl stop tobas_real.target", true) != ssh::SshClient::kNoError) {
    progress.close();
    clearExpectedTelemetryLoss();
    disconnectFromFlightController();
    qt::qErrorBox(this, "Failed to stop Tobas real service:\n\n" + QString(ssh_client_->errorMessage()));
    return;
  }
  progress.progressStep();

  // Load environment variables; continue even if they cannot be loaded.
  progress.setLabelText("Getting environment variables.");
  std::string project_env_text;
  if (ssh_client_->sftpRead(kProjectEnvPath, project_env_text, true) == ssh::SshClient::kNoError) {
    if (!project_env_parser_.parseFromText(project_env_text)) {
      progress.close();
      disconnectFromFlightController();
      qt::qErrorBox(this, "Failed to parse configuration file.");
      return;
    }
  }
  else {
    qWarning() << "Failed to get the current environment variables: " << ssh_client_->errorMessage();
  }
  progress.progressStep();

  // Use a clean build when packages change to avoid conflicts.
  if (config_pkg_name != project_env_parser_.config_pkg) {
    // Initialize the workspace.
    progress.setLabelText("Initializing colcon workspace.");
    if (ssh_client_->execute(std::format("rm -rf {}", kColconWSPathRoot), true)) {
      progress.close();
      disconnectFromFlightController();
      qt::qErrorBox(this, "Failed to remove the old colcon workspace:\n\n" + QString(ssh_client_->errorMessage()));
      return;
    }
    if (ssh_client_->execute(std::format("mkdir -p {}/src", kColconWSPathRoot), true)) {
      progress.close();
      disconnectFromFlightController();
      qt::qErrorBox(this, "Failed to create a new colcon workspace:\n\n" + QString(ssh_client_->errorMessage()));
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
  project_env_parser_.nic = network_config_.interface;
  project_env_parser_.id = kIdPrefix + std::to_string(currentId());
  if (ssh_client_->sftpWrite(kProjectEnvPath, project_env_parser_.exportText(), true) != ssh::SshClient::kNoError) {
    progress.close();
    disconnectFromFlightController();
    qt::qErrorBox(this, "Failed to set environment variables:\n\n" + QString(ssh_client_->errorMessage()));
    return;
  }
  progress.progressStep();

  // Send the project.
  progress.setLabelText("Sending the Tobas project to the flight controller.");
  const auto remote_dir = fs::path(kColconWSPathRoot) / "src/";
  const auto mesh_path = proj_paths.cfgMeshDirPath();
  const auto git_path = proj_paths.getProjPath() / ".git";
  if (ssh_client_->scpPut(proj_path, remote_dir, true, { mesh_path, git_path }, true) != ssh::SshClient::kNoError) {
    progress.close();
    disconnectFromFlightController();
    qt::qErrorBox(this, "Failed to send Tobas project:\n\n" + QString(ssh_client_->errorMessage()));
    return;
  }
  progress.progressStep();

  // Build the project on another thread so the GUI does not stop.
  progress.setLabelText("Building the Tobas project.");
  if (!remote_proj_builder_->build(proj_paths.remoteProjPath())) {
    progress.close();
    disconnectFromFlightController();
    const QString error_msg(remote_proj_builder_->getErrorMessage());
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
    return;
  }
  progress.progressStep();

  // Update the DDS settings.
  progress.setLabelText("Writing DDS configuration.");
  cyclonedds::Data dds_data;
  dds_data.interfaces.emplace_back("lo", 1, true);  // Multicast must be enabled to bridge the two NICs.
  dds_data.interfaces.emplace_back(network_config_.interface, 0, true);
  const auto dds_config_if_text = cyclonedds::exportText(dds_data);
  if (ssh_client_->sftpWrite(kCycloneddsConfigPath, dds_config_if_text, true) != ssh::SshClient::kNoError) {
    progress.close();
    disconnectFromFlightController();
    qt::qErrorBox(this, "Failed to write DDS configuration:\n\n" + QString(ssh_client_->errorMessage()));
    return;
  }
  progress.progressStep();

  // Enable the service.
  progress.setLabelText("Enabling the flight controller.");
  if (ssh_client_->execute("systemctl enable tobas_real.target", true) != ssh::SshClient::kNoError) {
    progress.close();
    disconnectFromFlightController();
    qt::qErrorBox(this, "Failed to enable Tobas real service:\n\n" + QString(ssh_client_->errorMessage()));
    return;
  }
  progress.progressStep();

  // Start the service.
  progress.setLabelText("Starting the flight controller.");
  if (ssh_client_->execute("systemctl start tobas_real.target", true) != ssh::SshClient::kNoError) {
    progress.close();
    disconnectFromFlightController();
    qt::qErrorBox(this, "Failed to start Tobas real service:\n\n" + QString(ssh_client_->errorMessage()));
    return;
  }
  progress.progressStep();

  // Wait for telemetry.
  progress.setLabelText("Waiting for telemetry.");
  if (!waitForHeartbeat()) {
    progress.close();
    disconnectFromFlightController();
    qt::qErrorBox(this, "Timed out waiting for a heartbeat from " + currentHost() + ".");
    return;
  }
  progress.progressStep();

  progress.close();
  connect_btn_->setChecked(true);
  qt::qInfoBox(this, "Tobas project is installed successfully.");
}

void GroundControlStationWidget::onFlightControllerScanFinished(
  const QVector<DiscoveredFlightController>& flight_controllers)
{
  if (connection_ready_) {
    return;
  }

  if (flight_controllers.isEmpty()) {
    setFlightControllerPlaceholder("No FC found");
    return;
  }

  const auto selected_host = currentHost();

  auto sorted_flight_controllers = flight_controllers;
  std::ranges::sort(
    sorted_flight_controllers,
    [](const auto& lhs, const auto& rhs)
    {
      if (lhs.hostname == rhs.hostname) {
        return lhs.address < rhs.address;
      }
      return lhs.hostname < rhs.hostname;
    });

  const QSignalBlocker block(fc_selector_);

  fc_selector_->clear();
  fc_selector_->addItem("Select FC...");
  for (const auto& flight_controller : sorted_flight_controllers) {
    const auto label = flight_controller.hostname + " (" + flight_controller.address + ')';
    fc_selector_->addItem(label);
    const auto index = fc_selector_->count() - 1;
    fc_selector_->setItemData(index, flight_controller.address, kHostRole);
  }

  // Preserve the current selection, or automatically select the first FC when none has been selected yet.
  const auto selected_index = fc_selector_->findData(selected_host, kHostRole);
  const auto next_index = selected_index > 0 ? selected_index : (selected_host.isEmpty() ? 1 : 0);
  fc_selector_->setCurrentIndex(next_index);

  updateHeaderActionAvailability();
}

void GroundControlStationWidget::onFlightControllerScanFailed(const QString& message)
{
  qWarning() << "Failed to scan for flight controllers:" << message;

  if (connection_ready_) {
    return;
  }

  if (fc_selector_->count() <= 1) {
    setFlightControllerPlaceholder("FC scan unavailable");
  }
}

void GroundControlStationWidget::onRestartButtonClicked()
{
  qDebug() << "GroundControlStationWidget::onRestartButtonClicked";

  // Confirm before restarting.
  if (!qt::yesOrNo(this, "Are you sure you want to restart " + currentConnectionDescription() + "?", qt::WARN)) {
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
}

void GroundControlStationWidget::onShutdownButtonClicked()
{
  qDebug() << "GroundControlStationWidget::onShutdownButtonClicked";

  // Confirm before shutting down.
  if (!qt::yesOrNo(this, "Are you sure you want to shut down " + currentConnectionDescription() + "?", qt::WARN)) {
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

  if (!connection_ready_) {
    return;
  }

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
