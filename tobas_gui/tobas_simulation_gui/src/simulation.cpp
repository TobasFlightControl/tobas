#include <QVBoxLayout>
#include <QHBoxLayout>

#include <tobas_path_tools/join.hpp>
#include <tobas_linux/errer.hpp>
#include <tobas_ros2_tools/register.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/widgets/progress_dialog.hpp>
#include <tobas_gui_common/constants.hpp>
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
  start_button_ = new QPushButton("Start");
  start_button_->setFixedSize(kButtonWidth, kButtonHeight);

  terminate_button_ = new QPushButton("Terminate");
  terminate_button_->setFixedSize(kButtonWidth, kButtonHeight);

  static_config_ = new StaticConfigWidget(node);
  dynamic_config_ = new DynamicConfigWidget(node);

  // Layout
  const auto button_cols = new QHBoxLayout();
  button_cols->addWidget(start_button_);
  button_cols->addWidget(terminate_button_);
  button_cols->addStretch();

  const auto config_cols = new QHBoxLayout();
  config_cols->addWidget(static_config_, 1);
  config_cols->addWidget(dynamic_config_, 1);

  const auto rows = new QVBoxLayout();
  rows->addLayout(button_cols);
  rows->addLayout(config_cols);

  setLayout(rows);

  // Connection
  connect(start_button_, &QPushButton::clicked, this, &self::onStartButtonClicked);
  connect(terminate_button_, &QPushButton::clicked, this, &self::onTerminateButtonClicked);

  reset();
  setEnabled(false);
}

SimulationWidget::~SimulationWidget()
{
  killGazeboLaunch();
}

bool SimulationWidget::updateTBSPath(const fs::path& tbs_path)
{
  if (!reset())
    return false;

  if (!drone_.load(common::getTBSDRNPath(tbs_path)))
  {
    qt::qErrorBox(this, "Failed to load drone configurations.");
    return false;
  }

  arming_sub_ = ros2::createSubscriber(
    node_, path::join(drone_.name, tobas::kRemoteIfaceTopicNS, tobas::kArmingTopic), &self::armingCb, this);

  tbs_path_ = tbs_path;
  setEnabled(true);

  return true;
}

void SimulationWidget::armingCb(const std_msgs::msg::Bool::ConstSharedPtr& arming)
{
  arming_ = arming;
}

bool SimulationWidget::reset()
{
  if (!killGazeboLaunch())
    return false;

  arming_ = nullptr;

  start_button_->setEnabled(true);
  terminate_button_->setEnabled(false);
  static_config_->setEnabled(true);
  dynamic_config_->setEnabled(false);

  return true;
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
  // プログレスバーを作成
  qt::ProgressDialog progress("Start Gazebo SITL", 4, this);
  progress.setCancelButton(nullptr);
  progress.show();

  // FCに接続できないことを確認
  progress.setLabelText("Checking SSH connection status.");
  if (ssh_client_.connect() == ssh::SSHClient::E_NO_ERROR)
  {
    qt::qWarnBox(this, "This operation cannot be performed while connecting to the flight controller.");
    progress.close();
    return false;
  }
  progress.progressStep();

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

  // 動的パラメータを初期化
  progress.setLabelText("Initializing dynamic configurations.");
  if (!initializeDynamicConfig())
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
  qt::ProgressDialog progress("Start Gazebo HITL", 8, this);
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
  const auto remote_dir = fs::path(common::kColconWSPathRemote) / "src/";
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

  // 動的パラメータを初期化
  progress.setLabelText("Initializing dynamic configurations.");
  if (!initializeDynamicConfig())
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
  qt::ProgressDialog progress("Terminate Gazebo HITL", 3, this);
  progress.setCancelButton(nullptr);
  progress.show();

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
  const auto config_pkg_name = common::getTBSConfigName(tbs_path_);
  const std::map<std::string, std::string> args{
    { "world_path", static_config_->worldPath().string() },
    { "launch_core", boolToText(launch_core) },
  };

  launch_pid_ = common::roslaunch(config_pkg_name, "gazebo.launch.xml", args);
  if (launch_pid_ < 0)
  {
    qt::qErrorBox(this, "Failed to launch Gazebo simulation.");
    return false;
  }

  RCLCPP_INFO_STREAM(node_->get_logger(), "Gazebo is launched with pid " << launch_pid_ << ".");
  return true;
}

bool SimulationWidget::initializeDynamicConfig()
{
  return dynamic_config_->initialize(drone_.name);
}

std::string SimulationWidget::boolToText(bool arg)
{
  if (arg)
    return "true";
  else
    return "false";
}

void SimulationWidget::onStartButtonClicked()
{
  bool success;
  switch (static_config_->simulationType())
  {
    case sim_type_t::SITL:
      success = startSITL();
      break;
    case sim_type_t::HITL:
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

  start_button_->setEnabled(false);
  terminate_button_->setEnabled(true);
  static_config_->setEnabled(false);
  dynamic_config_->setEnabled(true);

  qt::qInfoBox(this, "Gazebo simulation has started successfully.");
}

void SimulationWidget::onTerminateButtonClicked()
{
  bool success;
  switch (static_config_->simulationType())
  {
    case sim_type_t::SITL:
      success = terminateSITL();
      break;
    case sim_type_t::HITL:
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

  start_button_->setEnabled(true);
  terminate_button_->setEnabled(false);
  static_config_->setEnabled(true);
  dynamic_config_->setEnabled(false);

  qt::qInfoBox(this, "Gazebo simulation has been terminated successfully.");
}
}  // namespace sim
}  // namespace gui
