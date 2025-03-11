#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QCloseEvent>

#include <tobas_kdl/kdl_parser.hpp>
#include <tobas_path_tools/join.hpp>
#include <tobas_linux/errer.hpp>
#include <tobas_ros2_tools/register.hpp>
#include <tobas_ros2_tools/util.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_qt_tools/widgets/progress_dialog.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/util.hpp>
#include <tobas_gui_common/package.hpp>
#include <tobas_gui_common/ros2_cli.hpp>

#include "tobas_simulation_gui/simulation.hpp"

namespace fs = std::filesystem;

namespace gui
{
namespace sim
{
SimulationWidget::SimulationWidget(rclcpp::Node::SharedPtr node)
  : node_(node), ssh_client_(node), remote_pkg_builder_(node)
{
  start_stop_button_ = new qt::ToggleButton("Start", "Terminate");
  start_stop_button_->setFixedSize(kButtonWidth, kButtonHeight);

  sim_settings_ = new SimulationSettingsWidget(node);
  dynamic_config_ = new DynamicConfigWidget(node);
  commanders_ = new CommandersWidget(node, tree_, drone_);

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
  setEnabled(false);
}

void SimulationWidget::reset()
{
  resetDynamicConfig();
  resetCommanders();

  if (launch_pid_ >= 0)
    killGazeboLaunch();

  arming_ = nullptr;

  start_stop_button_->setChecked(false);

  sim_settings_->setEnabled(true);
  dynamic_config_->setEnabled(false);
  commanders_->setEnabled(false);
}

bool SimulationWidget::updateTBSPath(const fs::path& tbs_path)
{
  reset();

  if (!kdl::treeFromFile(common::getOriginalURDFPath(tbs_path), tree_))
  {
    qt::qErrorBox(this, "Failed to load kdl tree.");
    return false;
  }

  if (!drone_.load(common::getTBSDRNPath(tbs_path)))
  {
    qt::qErrorBox(this, "Failed to load drone configurations.");
    return false;
  }

  const auto& ns = drone_.name;

  dynamic_config_->updateNamespace(ns);
  commanders_->updateInternalDataStructures();

  arming_sub_ = ros2::createSubscriber(
    node_, path::join(ns, tobas::kRemoteIfaceTopicNS, tobas::kArmingTopic), &self::armingCb, this);

  tbs_path_ = tbs_path;
  setEnabled(true);

  return true;
}

void SimulationWidget::closeEvent(QCloseEvent* event)
{
  RCLCPP_DEBUG(node_->get_logger(), "SimulationWidget::closeEvent");

  // 親ウィジェットを閉じるときに子プロセスを破棄
  if (launch_pid_ >= 0)
    killGazeboLaunch();

  event->accept();
}

void SimulationWidget::armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming)
{
  arming_ = arming;
}

bool SimulationWidget::killGazeboLaunch()
{
  if (launch_pid_ < 0)
  {
    RCLCPP_INFO(node_->get_logger(), "Gazebo simulation is not running.");
    return true;
  }

  if (kill(launch_pid_, SIGINT) != 0)
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Failed to kill child process: " << linux::strError());
    return false;
  }

  if (!cmd_executor_.execute("ps aux | grep \"gz sim\" | grep -v grep | awk '{ print \"kill -9\", $2 }' | sh"))
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Failed to kill Gazebo: " << cmd_executor_.getOutput());
    return false;
  }

  launch_pid_ = -1;
  return true;
}

bool SimulationWidget::startSITL()
{
  // フライトコードが起動していないことを確認
  if (arming_)
  {
    qt::qWarnBox(this, "This operation cannot be performed while flight controller is active.");
    return false;
  }

  // プログレスバーを作成
  qt::ProgressDialog progress("Start Gazebo SITL", 4, this);
  progress.setCancelButton(nullptr);
  progress.show();

  // Tobasパッケージをビルド
  progress.setLabelText("Building Tobas package.");
  if (!buildLocalPackage())
  {
    progress.close();
    return false;
  }
  progress.progressStep();

  // Gazeboを起動
  progress.setLabelText("Launching Gazebo simulation.");
  if (!launchGazebo(true))
  {
    progress.close();
    return false;
  }
  progress.progressStep();

  // 動的パラメータを起動
  progress.setLabelText("Starting dynamic configurations.");
  if (!startDynamicConfig())
  {
    progress.close();
    return false;
  }
  progress.progressStep();

  // コマンダーを起動
  progress.setLabelText("Starting commanders.");
  if (!startCommanders())
  {
    progress.close();
    return false;
  }
  progress.progressStep();

  progress.close();
  return true;
}

bool SimulationWidget::terminateSITL()
{
  // 動的パラメータを終了
  resetDynamicConfig();

  // コマンダーを終了
  resetCommanders();

  // Gazeboプロセスを終了
  if (!killGazeboLaunch())
  {
    qt::qErrorBox(this, "Failed to kill Gazebo process.");
    return false;
  }

  return true;
}

bool SimulationWidget::startHITL()
{
  // アームされていないことを確認
  if (arming_ == nullptr)
  {
    qt::qWarnBox(
      this, "This operation cannot be performed because the arming status is not received from the flight controller.");
    return false;
  }
  else
  {
    if (arming_->data)
    {
      qt::qWarnBox(this, "This operation cannot be performed while the rotors are armed.");
      return false;
    }
  }

  // プログレスバーを作成
  qt::ProgressDialog progress("Start Gazebo HITL", 9, this);
  progress.setCancelButton(nullptr);
  progress.show();

  // ローカルパッケージをビルド
  progress.setLabelText("Building Tobas local package.");
  if (!buildLocalPackage())
  {
    progress.close();
    return false;
  }
  progress.progressStep();

  // SSH接続
  progress.setLabelText("Connecting to the flight controller.");
  if (ssh_client_.connect() != ssh::SSHClient::E_NO_ERROR)
  {
    qt::qErrorBox(this, "No SSH connection: " + QString(ssh_client_.errorMessage()));
    progress.close();
    return false;
  }
  progress.progressStep();

  // Realサービスを停止
  progress.setLabelText("Stopping Tobas real service.");
  if (ssh_client_.execute("systemctl stop tobas_real.target", true) != ssh::SSHClient::E_NO_ERROR)
  {
    qt::qErrorBox(this, "Failed to stop Tobas real service:\n\n" + QString(ssh_client_.errorMessage()));
    progress.close();
    return false;
  }
  progress.progressStep();

  // Tobasパッケージを送信
  progress.setLabelText("Sending Tobas configuration package to the flight controller.");
  const auto mesh_path = common::getMeshPath(tbs_path_);
  const auto remote_dir = fs::path(tobas::kColconWSPathRoot) / "src/";
  if (ssh_client_.scpPut(tbs_path_, remote_dir, { mesh_path }, true) != ssh::SSHClient::E_NO_ERROR)
  {
    qt::qErrorBox(this, "Failed to send Tobas configuration package:\n\n" + QString(ssh_client_.errorMessage()));
    progress.close();
    return false;
  }
  progress.progressStep();

  // リモートパッケージをビルド
  progress.setLabelText("Building Tobas remote package.");
  const auto remote_tbs_path = common::getRemoteTBSPath(tbs_path_);
  if (!remote_pkg_builder_.build(remote_tbs_path))
  {
    qt::qErrorBox(
      this,
      "Failed to build the Tobas remote package:\n\n" + QString::fromStdString(remote_pkg_builder_.getErrorMessage()));
    progress.close();
    return false;
  }
  progress.progressStep();

  // Gazeboを起動
  progress.setLabelText("Launching Gazebo simulation.");
  if (!launchGazebo(true))  // FIXME: 通信負荷を改善してcoreをRPi側で立ち上げる
  {
    progress.close();
    return false;
  }
  progress.progressStep();

  // HITLサービスを起動
  progress.setLabelText("Starting Tobas HITL service.");
  if (ssh_client_.execute("systemctl restart tobas_hitl.service", true) != ssh::SSHClient::E_NO_ERROR)
  {
    qt::qErrorBox(this, "Failed to restart Tobas HITL service:\n\n" + QString(ssh_client_.errorMessage()));
    progress.close();
    return false;
  }
  progress.progressStep();

  // 動的パラメータを起動
  progress.setLabelText("Starting dynamic configurations.");
  if (!startDynamicConfig())
  {
    progress.close();
    return false;
  }
  progress.progressStep();

  // コマンダーを起動
  progress.setLabelText("Starting commanders.");
  if (!startCommanders())
  {
    progress.close();
    return false;
  }
  progress.progressStep();

  progress.close();
  return true;
}

bool SimulationWidget::terminateHITL()
{
  // プログレスバーを作成
  qt::ProgressDialog progress("Terminate Gazebo HITL", 5, this);
  progress.setCancelButton(nullptr);
  progress.show();

  // 動的パラメータを終了
  progress.setLabelText("Terminating dynamic configurations.");
  resetDynamicConfig();
  progress.progressStep();

  // コマンダーを終了
  progress.setLabelText("Terminating commanders.");
  resetCommanders();
  progress.progressStep();

  // Gazeboプロセスを終了
  progress.setLabelText("Terminating Gazebo simulation.");
  if (!killGazeboLaunch())
  {
    qt::qErrorBox(this, "Failed to kill Gazebo process.");
    return false;
  }
  progress.progressStep();

  // HITLサービスを停止
  progress.setLabelText("Stopping Tobas HITL service.");
  if (ssh_client_.execute("systemctl stop tobas_hitl.service", true) != ssh::SSHClient::E_NO_ERROR)
  {
    qt::qErrorBox(this, "Failed to stop Tobas HITL service:\n\n" + QString(ssh_client_.errorMessage()));
    progress.close();
    return false;
  }
  progress.progressStep();

  // Realサービスを起動
  progress.setLabelText("Starting Tobas real service.");
  if (ssh_client_.execute("systemctl restart tobas_real.target", true) != ssh::SSHClient::E_NO_ERROR)
  {
    qt::qErrorBox(this, "Failed to start Tobas real service:\n\n" + QString(ssh_client_.errorMessage()));
    progress.close();
    return false;
  }
  progress.progressStep();

  progress.close();
  return true;
}

bool SimulationWidget::buildLocalPackage()
{
  if (!local_pkg_builder_.build(tbs_path_))
  {
    qt::qErrorBox(
      this, "Failed to build Tobas local package:\n\n" + QString::fromStdString(local_pkg_builder_.getOutput()));
    return false;
  }

  return true;
}

bool SimulationWidget::launchGazebo(bool launch_core)
{
  const auto install_path = ros2::expandUser(tobas::kColconWSPathHome) / "install";
  const auto config_pkg_name = common::getTBSConfigName(tbs_path_);
  const std::map<std::string, std::string> args{
    { "world_path", sim_settings_->worldPath().string() },
    { "launch_core", boolToText(launch_core) },
  };

  launch_pid_ = common::roslaunch(install_path, config_pkg_name, "gazebo.launch.xml", args);
  if (launch_pid_ < 0)
  {
    qt::qErrorBox(this, "Failed to launch Gazebo simulation.");
    return false;
  }

  RCLCPP_INFO_STREAM(node_->get_logger(), "Gazebo is launched with pid " << launch_pid_ << ".");
  return true;
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
  if (arg)
    return "true";
  else
    return "false";
}

void SimulationWidget::onStartRequested()
{
  bool success;
  switch (sim_settings_->loopType())
  {
    case loop_type_t::SITL:
      success = startSITL();
      break;
    case loop_type_t::HITL:
      success = startHITL();
      break;
    default:
      throw;
  }

  if (!success)
  {
    reset();
    return;
  }

  sim_settings_->setEnabled(false);
  dynamic_config_->setEnabled(true);
  commanders_->setEnabled(true);

  qt::qInfoBox(this, "Gazebo simulation has started successfully.");

  Q_EMIT started();
}

void SimulationWidget::onTerminateRequested()
{
  bool success;
  switch (sim_settings_->loopType())
  {
    case loop_type_t::SITL:
      success = terminateSITL();
      break;
    case loop_type_t::HITL:
      success = terminateHITL();
      break;
    default:
      throw;
  }

  if (!success)
  {
    reset();
    return;
  }

  // シミュレーションのアーム状態が入っているのでリセット
  arming_ = nullptr;

  sim_settings_->setEnabled(true);
  dynamic_config_->setEnabled(false);
  commanders_->setEnabled(false);

  qt::qInfoBox(this, "Gazebo simulation has been terminated successfully.");

  Q_EMIT terminated();
}
}  // namespace sim
}  // namespace gui
