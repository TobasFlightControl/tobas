#include "tobas_simulation_gui/simulation.hpp"

#include <QApplication>
#include <QCloseEvent>
#include <QDebug>
#include <QHBoxLayout>
#include <QVBoxLayout>

#include <tobas_constants/constants.hpp>
#include <tobas_gui_common/local_project_builder.hpp>
#include <tobas_gui_common/remote_project_builder.hpp>
#include <tobas_gui_common/ros2_cli.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/util.hpp>
#include <tobas_qt_tools/widgets/progress_dialog.hpp>
#include <tobas_qt_tools/widgets/wait_spinner.hpp>
#include <tobas_std_tools/check.hpp>

#include "tobas_simulation_gui/kill_gazebo_thread.hpp"

namespace fs = std::filesystem;

namespace gui
{
namespace sim
{
SimulationWidget::SimulationWidget(rclcpp::Node::SharedPtr node, const RosQtBridge& bridge)
  : node_(node), ssh_client_(node)
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
  resetDynamicConfig();
  resetCommanders();

  if (isRunning()) {
    killGazebo();
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
    killGazebo();
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
  qt::ProgressDialog progress("Start SITL", 4, this);
  progress.setCancelButton(nullptr);
  progress.show();

  // Tobasパッケージをビルド
  progress.setLabelText("Building the Tobas project packages.");
  const auto build_res = cmn::buildLocalProjectBackground(proj_paths_.getProjPath());
  if (!build_res) {
    qt::qErrorBox(this, "Failed to build the Tobas project:\n\n" + build_res.error());
    progress.close();
    return false;
  }
  progress.progressStep();

  // Gazeboを起動
  progress.setLabelText("Launching Gazebo.");
  if (!launchGazebo(true)) {
    progress.close();
    reset();
    return false;
  }
  progress.progressStep();

  // 動的パラメータを起動
  progress.setLabelText("Starting dynamic configurations.");
  if (!startDynamicConfig()) {
    progress.close();
    reset();
    return false;
  }
  progress.progressStep();

  // コマンダーを起動
  progress.setLabelText("Starting commanders.");
  if (!startCommanders()) {
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
  resetDynamicConfig();

  // コマンダーを終了
  resetCommanders();

  // 別スレッドでGazeboプロセスを終了
  const auto kill_gazebo_res = killGazebo();

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
  qt::ProgressDialog progress("Start HITL", 9, this);
  progress.setCancelButton(nullptr);
  progress.show();

  // ローカルパッケージをビルド
  progress.setLabelText("Building the Tobas local project.");
  const auto local_build_res = cmn::buildLocalProjectBackground(proj_paths_.getProjPath());
  if (!local_build_res) {
    qt::qErrorBox(this, "Failed to build the Tobas local project:\n\n" + local_build_res.error());
    progress.close();
    return false;
  }
  progress.progressStep();

  // SSH接続
  progress.setLabelText("Connecting to the flight controller.");
  if (ssh_client_.connect() != ssh::SSHClient::kNoError) {
    qt::qErrorBox(this, "No SSH connection: " + QString(ssh_client_.errorMessage()));
    progress.close();
    return false;
  }
  progress.progressStep();

  // Realサービスを停止
  progress.setLabelText("Stopping the Tobas real service.");
  if (ssh_client_.execute("systemctl stop tobas_real.target", true) != ssh::SSHClient::kNoError) {
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
  if (ssh_client_.scpPut(proj_path, remote_dir, true, { mesh_path }, true) != ssh::SSHClient::kNoError) {
    qt::qErrorBox(this, "Failed to send Tobas project:\n\n" + QString(ssh_client_.errorMessage()));
    progress.close();
    return false;
  }
  progress.progressStep();

  // リモートパッケージをビルド
  progress.setLabelText("Building the Tobas remote project.");
  const auto remote_build_res = cmn::buildRemoteProjectBackground(node_, proj_paths_.remoteProjPath());
  if (!remote_build_res) {
    qt::qErrorBox(this, "Failed to build the Tobas remote project:\n\n" + remote_build_res.error());
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

  // HITLサービスを起動
  progress.setLabelText("Starting the Tobas HITL service.");
  if (ssh_client_.execute("systemctl restart tobas_hitl.service", true) != ssh::SSHClient::kNoError) {
    qt::qErrorBox(this, "Failed to restart Tobas HITL service:\n\n" + QString(ssh_client_.errorMessage()));
    progress.close();
    return false;
  }
  progress.progressStep();

  // 動的パラメータを起動
  progress.setLabelText("Starting dynamic configurations.");
  if (!startDynamicConfig()) {
    progress.close();
    return false;
  }
  progress.progressStep();

  // コマンダーを起動
  progress.setLabelText("Starting commanders.");
  if (!startCommanders()) {
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
  const auto kill_gazebo_res = killGazebo(false);
  if (!kill_gazebo_res) {
    qt::qErrorBox(this, "Failed to kill the simulation process:\n" + kill_gazebo_res.error());
    progress.close();
    reset();
    return;
  }
  progress.progressStep();

  // HITLサービスを停止
  progress.setLabelText("Stopping the Tobas HITL service.");
  if (ssh_client_.execute("systemctl stop tobas_hitl.service", true) != ssh::SSHClient::kNoError) {
    qt::qErrorBox(this, "Failed to stop Tobas HITL service:\n\n" + QString(ssh_client_.errorMessage()));
    progress.close();
    reset();
    return;
  }
  progress.progressStep();

  // Realサービスを起動
  progress.setLabelText("Starting the Tobas real service.");
  if (ssh_client_.execute("systemctl restart tobas_real.target", true) != ssh::SSHClient::kNoError) {
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

bool SimulationWidget::launchGazebo(bool launch_core)
{
  const auto config_pkg_name = proj_paths_.cfgPkgName();
  const std::map<std::string, std::string> args{
    { "world_path", sim_settings_->worldPath().string() },
    { "user_debug", std::format("{}", sim_settings_->userDebug()) },
    { "launch_core", boolToText(launch_core) },
    { "x", std::to_string(sim_settings_->x()) },
    { "y", std::to_string(sim_settings_->y()) },
    { "z", std::to_string(sim_settings_->z()) },
    { "roll", std::to_string(sim_settings_->roll()) },
    { "pitch", std::to_string(sim_settings_->pitch()) },
    { "yaw", std::to_string(sim_settings_->yaw()) },
  };

  launch_pid_ = cmn::roslaunch(config_pkg_name, "gazebo.launch.xml", args);
  if (launch_pid_ < 0) {
    qt::qErrorBox(this, "Failed to start Gazebo process.");
    return false;
  }

  qInfo() << "Simulation has been started with pid " << launch_pid_ << ".";
  return true;
}

std::expected<void, QString> SimulationWidget::killGazebo(bool run_spinner)
{
  // スレッドを作成
  KillGazeboThread thread(node_, launch_pid_);

  // 別スレッドの結果をキャッチするためのイベントループを用意
  bool success;
  QString message;
  QEventLoop loop;
  connect(
    &thread,
    &KillGazeboThread::finished,
    [&success, &message, &loop](bool _success, const QString& _message)
    {
      success = _success;
      message = _message;
      loop.quit();
    });

  // スピナーを起動
  qt::WaitSpinnerWidget spinner(Qt::WindowModal, this);
  if (run_spinner) {
    spinner.show();
    spinner.start();
  }

  // イベントループを回しながらスレッドが終了するまで待機
  thread.start();
  loop.exec();
  thread.wait();

  // スピナーを停止
  if (run_spinner) {
    spinner.hide();
    spinner.stop();
  }

  // 別スレッドの結果を返す
  if (success) {
    return {};
  }
  else {
    return std::unexpected(message);
  }
}

bool SimulationWidget::startDynamicConfig()
{
  return dynamic_config_->start();
}

void SimulationWidget::resetDynamicConfig()
{
  dynamic_config_->reset();
}

bool SimulationWidget::startCommanders()
{
  return commanders_->start();
}

void SimulationWidget::resetCommanders()
{
  commanders_->reset();
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
