#include "tobas_simulation_gui/simulation.hpp"

#include <QApplication>
#include <QCloseEvent>
#include <QDebug>
#include <QHBoxLayout>
#include <QVBoxLayout>

#include <tobas_constants/constants.hpp>
#include <tobas_gui_common/local_project_builder.hpp>
#include <tobas_gui_common/ros2_cli.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/util.hpp>
#include <tobas_qt_tools/widgets/progress_dialog.hpp>
#include <tobas_std_tools/check.hpp>

#include "tobas_simulation_gui/gazebo.hpp"

namespace fs = std::filesystem;

namespace gui
{
namespace sim
{
SimulationWidget::SimulationWidget(rclcpp::Node::SharedPtr node, const RosQtBridge& bridge)
  : node_(node), ssh_client_(node), remote_proj_builder_(node), spinner_(Qt::WindowModal, this)
{
  start_stop_button_ = new qt::ToggleButton("Start", "Terminate");
  start_stop_button_->setFixedSize(kButtonWidth, kButtonHeight);

  sim_settings_ = new SimulationSettingsWidget(node);
  dynamic_config_ = new DynamicConfigWidget(node);
  commanders_ = new CommandersWidget(node, bridge, tree_, drone_);

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
  connect(&bridge, &RosQtBridge::armingReceived, this, &self::armingCb, Qt::QueuedConnection);

  reset();
  setEnabled(false);
}

void SimulationWidget::reset()
{
  dynamic_config_->reset();
  commanders_->reset();

  if (isRunning()) {
    killGazeboWithSpinner();
    launch_pid_ = -1;
    qWarning() << "Gazebo was forcibly shut down.";
  }

  arming_.reset();

  start_stop_button_->setChecked(false);

  sim_settings_->setEnabled(true);
  dynamic_config_->setEnabled(false);
  commanders_->setEnabled(false);
}

bool SimulationWidget::updateProject(const fs::path& proj_path)
{
  reset();

  // Update project path
  proj_paths_.setProjPath(proj_path);

  // Load KDL tree
  const auto uadf_path = proj_paths_.originalUadfPath();
  if (!uadf_parser_.parseFromPath(uadf_path, uadf_)) {
    qt::qErrorBox(this, "Failed to parse UADF:\n\n" + QString::fromStdString(uadf_parser_.errorMessage()));
    return false;
  }
  if (!tree_parser_.parseFromUrdf(*uadf_.urdf, tree_)) {
    qt::qErrorBox(
      this, "Failed to construct KDL tree from URDF:\n\n" + QString::fromStdString(tree_parser_.errorMessage()));
    return false;
  }

  // Load drone configuration
  const auto tbsdrn_path = proj_paths_.tbsdrnPath();
  if (!drone_.load(tbsdrn_path)) {
    qt::qErrorBox(this, "Failed to load drone configuration.");
    return false;
  }

  dynamic_config_->updateNamespace(drone_.name);
  commanders_->updateInternalDataStructures();

  setEnabled(true);

  return true;
}

bool SimulationWidget::isRunning() const
{
  return launch_pid_ >= 0;
}

void SimulationWidget::closeEvent(QCloseEvent* event)
{
  // 親ウィジェットを閉じるときに子プロセスを破棄
  if (isRunning()) {
    killGazeboWithSpinner();
  }

  event->accept();
}

bool SimulationWidget::startSITL()
{
  // フライトコードが起動していないことを確認
  if (arming_) {
    qt::qWarnBox(this, "This operation cannot be performed while flight controller is active.");
    return false;
  }

  // プログレスバーを作成
  qt::ProgressDialog progress("Start SITL", 6, this);
  progress.setCancelButton(nullptr);
  progress.show();

  // Tobasパッケージをビルド
  progress.setLabelText("Building the Tobas project packages.");
  const auto build_res = cmn::buildLocalProject(proj_paths_.getProjPath());
  if (!build_res) {
    qt::qErrorBox(this, "Failed to build the Tobas project:\n\n" + build_res.error());
    progress.close();
    return false;
  }
  progress.progressStep();

  // Gazeboを起動
  progress.setLabelText("Launching the simulation.");
  if (!launchGazebo(true)) {
    progress.close();
    reset();
    return false;
  }
  progress.progressStep();

  // Gazeboサーバの起動を待つ
  progress.setLabelText("Waiting for the Gazebo server to start.");
  if (!waitUntilGazeboServerReady()) {
    qt::qErrorBox(this, "Failed to start the Gazebo server.");
    progress.close();
    reset();
    return false;
  }
  progress.progressStep();

  // Gazeboレンダリングの開始を待つ
  progress.setLabelText("Waiting for Gazebo rendering to start.");
  if (!waitUntilGazeboRenderingReady()) {
    qt::qErrorBox(this, "Failed to get the Gazebo rendering information.");
    progress.close();
    reset();
    return false;
  }
  progress.progressStep();

  // 動的パラメータを起動
  progress.setLabelText("Starting dynamic configuration.");
  if (!dynamic_config_->start()) {
    progress.close();
    reset();
    return false;
  }
  progress.progressStep();

  // コマンダーを起動
  progress.setLabelText("Starting commanders.");
  if (!commanders_->start()) {
    progress.close();
    reset();
    return false;
  }
  progress.progressStep();

  progress.close();
  return true;
}

void SimulationWidget::terminateSITL()
{
  // 動的パラメータを終了
  dynamic_config_->reset();

  // コマンダーを終了
  commanders_->reset();

  // 別スレッドでGazeboプロセスを終了
  const auto kill_gazebo_res = killGazeboWithSpinner();

  // シミュレーションが正常に終了できなければアプリケーション全体を落とす
  if (!kill_gazebo_res) {
    qt::qErrorBox(this, kill_gazebo_res.error());
    QApplication::quit();
    return;
  }

  reset();
  Q_EMIT terminated();
  qt::qInfoBox(this, "SITL has been terminated successfully.");
}

bool SimulationWidget::startHITL()
{
  // アームされていないことを確認
  if (!arming_) {
    qt::qWarnBox(
      this, "This operation cannot be performed because the arming status is not received from the flight controller.");
    return false;
  }
  else {
    if (arming_->data) {
      qt::qWarnBox(this, "This operation cannot be performed while the rotors are armed.");
      return false;
    }
  }

  // プログレスバーを作成
  qt::ProgressDialog progress("Start HITL", 11, this);
  progress.setCancelButton(nullptr);
  progress.show();

  // ローカルパッケージをビルド
  progress.setLabelText("Building the Tobas project.");
  const auto local_build_res = cmn::buildLocalProject(proj_paths_.getProjPath());
  if (!local_build_res) {
    qt::qErrorBox(this, "Failed to build the Tobas project:\n\n" + local_build_res.error());
    progress.close();
    return false;
  }
  progress.progressStep();

  // SSH接続
  progress.setLabelText("Connecting to the flight controller.");
  if (ssh_client_.connect() != ssh::SshClient::kNoError) {
    qt::qErrorBox(this, "No SSH connection: " + QString(ssh_client_.errorMessage()));
    progress.close();
    return false;
  }
  progress.progressStep();

  // Realサービスを停止
  progress.setLabelText("Stopping the Tobas real service.");
  if (ssh_client_.execute("systemctl stop tobas_real.target", true) != ssh::SshClient::kNoError) {
    qt::qErrorBox(this, "Failed to stop Tobas real service:\n\n" + QString(ssh_client_.errorMessage()));
    progress.close();
    return false;
  }
  progress.progressStep();

  // Tobasパッケージを送信
  progress.setLabelText("Sending the Tobas project to the flight controller.");
  const auto& proj_path = proj_paths_.getProjPath();
  const auto mesh_path = proj_paths_.cfgMeshDirPath();
  const auto remote_dir = fs::path(tobas::kColconWSPathRoot) / "src/";
  if (ssh_client_.scpPut(proj_path, remote_dir, true, { mesh_path }, true) != ssh::SshClient::kNoError) {
    qt::qErrorBox(this, "Failed to send Tobas project:\n\n" + QString(ssh_client_.errorMessage()));
    progress.close();
    return false;
  }
  progress.progressStep();

  // リモートパッケージをビルド
  progress.setLabelText("Building the Tobas project.");
  if (!remote_proj_builder_.build(proj_paths_.remoteProjPath())) {
    qt::qErrorBox(this, "Failed to build the Tobas project:\n\n" + QString(remote_proj_builder_.getErrorMessage()));
    progress.close();
    return false;
  }
  progress.progressStep();

  // Gazeboを起動
  progress.setLabelText("Launching Gazebo.");
  if (!launchGazebo(true))  // FIXME: 通信負荷を改善してcoreをRPi側で立ち上げる
  {
    progress.close();
    return false;
  }
  progress.progressStep();

  // Gazeboサーバの起動を待つ
  progress.setLabelText("Waiting for the Gazebo server to start.");
  if (!waitUntilGazeboServerReady()) {
    progress.close();
    return false;
  }
  progress.progressStep();

  // Gazeboレンダリングの開始を待つ
  progress.setLabelText("Waiting for Gazebo rendering to start.");
  if (!waitUntilGazeboRenderingReady()) {
    progress.close();
    return false;
  }
  progress.progressStep();

  // HITLサービスを起動
  progress.setLabelText("Starting the Tobas HITL service.");
  if (ssh_client_.execute("systemctl restart tobas_hitl.service", true) != ssh::SshClient::kNoError) {
    qt::qErrorBox(this, "Failed to restart Tobas HITL service:\n\n" + QString(ssh_client_.errorMessage()));
    progress.close();
    return false;
  }
  progress.progressStep();

  // 動的パラメータを起動
  progress.setLabelText("Starting dynamic configuration.");
  if (!dynamic_config_->start()) {
    progress.close();
    return false;
  }
  progress.progressStep();

  // コマンダーを起動
  progress.setLabelText("Starting commanders.");
  if (!commanders_->start()) {
    progress.close();
    return false;
  }
  progress.progressStep();

  progress.close();
  return true;
}

void SimulationWidget::terminateHITL()
{
  // プログレスバーを作成
  qt::ProgressDialog progress("Terminate HITL", 3, this);
  progress.setCancelButton(nullptr);
  progress.show();

  // launchを終了
  progress.setLabelText("Terminating the simulation.");
  const auto kill_gazebo_res = killGazebo(node_, launch_pid_);
  if (!kill_gazebo_res) {
    qt::qErrorBox(this, kill_gazebo_res.error());
    progress.close();
    reset();
    return;
  }
  progress.progressStep();

  // HITLサービスを停止
  progress.setLabelText("Stopping the Tobas HITL service.");
  if (ssh_client_.execute("systemctl stop tobas_hitl.service", true) != ssh::SshClient::kNoError) {
    qt::qErrorBox(this, "Failed to stop Tobas HITL service:\n\n" + QString(ssh_client_.errorMessage()));
    progress.close();
    reset();
    return;
  }
  progress.progressStep();

  // Realサービスを起動
  progress.setLabelText("Starting the Tobas real service.");
  if (ssh_client_.execute("systemctl restart tobas_real.target", true) != ssh::SshClient::kNoError) {
    qt::qErrorBox(this, "Failed to start Tobas real service:\n\n" + QString(ssh_client_.errorMessage()));
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

std::map<std::string, std::string> SimulationWidget::makeGazeboLaunchArguments(bool launch_core) const
{
  std::map<std::string, std::string> args{
    { "user_debug", std::format("{}", sim_settings_->userDebug()) },
    { "launch_core", boolToText(launch_core) },
    { "x", std::to_string(sim_settings_->x()) },
    { "y", std::to_string(sim_settings_->y()) },
    { "z", std::to_string(sim_settings_->z()) },
    { "roll", std::to_string(sim_settings_->roll()) },
    { "pitch", std::to_string(sim_settings_->pitch()) },
    { "yaw", std::to_string(sim_settings_->yaw()) },
  };

  const auto world_path = sim_settings_->worldPath().string();
  if (!world_path.empty()) {
    args["world_path"] = world_path;
  }

  const auto sbus_device = sim_settings_->sbusDevicePath().string();
  if (!sbus_device.empty()) {
    args["sbus_device"] = sbus_device;
  }

  return args;
}

bool SimulationWidget::launchGazebo(bool launch_core)
{
  const auto args = makeGazeboLaunchArguments(launch_core);
  launch_pid_ = cmn::roslaunch(proj_paths_.cfgPkgName(), "gazebo.launch.xml", args);
  if (launch_pid_ < 0) {
    qt::qErrorBox(this, "Failed to start Gazebo process.");
    return false;
  }

  qInfo() << "Simulation has been started with pid " << launch_pid_ << ".";
  return true;
}

std::expected<void, QString> SimulationWidget::killGazeboWithSpinner()
{
  spinner_.start();
  const auto res = killGazebo(node_, launch_pid_);
  spinner_.stop();
  return res;
}

std::string SimulationWidget::boolToText(bool arg)
{
  if (arg) {
    return "true";
  }
  else {
    return "false";
  }
}

void SimulationWidget::onStartRequested()
{
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

void SimulationWidget::armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming)
{
  arming_ = arming;
}
}  // namespace sim
}  // namespace gui
