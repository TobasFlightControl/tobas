#include <rcutils/env.h>
#include <QLabel>
#include <QButtonGroup>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QCoreApplication>

#include <tobas_path_tools/join.hpp>
#include <tobas_kdl/kdl_parser.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/widgets/stacked_widget.hpp>
#include <tobas_qt_tools/widgets/progress_dialog.hpp>
#include <tobas_gui_common/constants.hpp>
#include <tobas_gui_common/package.hpp>

#include "tobas_gui_core/gui_core.hpp"
#include "tobas_gui_core/constants.hpp"
#include "tobas_gui_core/app_button.hpp"

namespace fs = std::filesystem;

namespace gui
{
namespace core
{
GUICoreWidget::GUICoreWidget(rclcpp::Node::SharedPtr node)
  : node_(node), property_client_(node, tobas::kPropertyServerName, kPkgName), ssh_client_(node), package_builder_(node)
{
  const auto pkg_path = fs::path(ament_index_cpp::get_package_share_directory(kPkgName));
  const auto rsrc_path = pkg_path / "resources";

  // Applications
  homepage_ = new homepage::HomepageWidget();
  urdf_builder_ = new URDFBuilder();
  setup_assistant_ = new setup_assistant::SetupAssistantWidget(node);
  hardware_setup_ = new hardware_setup::HardwareSetupWidget(node, tree_, drone_);
  control_system_ = new control_system::ControlSystemWidget(node, drone_);
  param_tuning_ = new param_tuning::ParameterTuningWidget(node);
  flight_log_ = new log::FlightLogWidget(node);
  simulation_ = new sim::SimulationWidget(node);

  // TODO: 別々のアイコンを設定
  const auto homepage_btn = new AppButton("Homepage", QString::fromStdString(rsrc_path / "icon.png"));
  const auto urdf_builder_btn = new AppButton("URDF Builder", QString::fromStdString(rsrc_path / "icon.png"));
  const auto setup_assistant_btn = new AppButton("Setup Assistant", QString::fromStdString(rsrc_path / "icon.png"));
  const auto hardware_setup_btn = new AppButton("Hardware Setup", QString::fromStdString(rsrc_path / "icon.png"));
  const auto control_system_btn = new AppButton("Control System", QString::fromStdString(rsrc_path / "icon.png"));
  const auto param_tuning_btn = new AppButton("Param Tuning", QString::fromStdString(rsrc_path / "icon.png"));
  const auto flight_log_btn = new AppButton("Flight Log", QString::fromStdString(rsrc_path / "icon.png"));
  const auto simulation_btn = new AppButton("Simulation", QString::fromStdString(rsrc_path / "icon.png"));

  const auto app_sw = new qt::StackedWidget();
  app_sw->addWidget(homepage_);
  app_sw->addWidget(urdf_builder_);
  app_sw->addWidget(setup_assistant_);
  app_sw->addWidget(hardware_setup_);
  app_sw->addWidget(control_system_);
  app_sw->addWidget(param_tuning_);
  app_sw->addWidget(flight_log_);
  app_sw->addWidget(simulation_);

  const auto btn_group = new QButtonGroup(this);
  int btn_id = 0;
  btn_group->addButton(homepage_btn, btn_id++);
  btn_group->addButton(urdf_builder_btn, btn_id++);
  btn_group->addButton(setup_assistant_btn, btn_id++);
  btn_group->addButton(hardware_setup_btn, btn_id++);
  btn_group->addButton(control_system_btn, btn_id++);
  btn_group->addButton(param_tuning_btn, btn_id++);
  btn_group->addButton(flight_log_btn, btn_id++);
  btn_group->addButton(simulation_btn, btn_id++);
  btn_group->buttons().first()->setChecked(true);

  // Package manager
  tbs_path_ = new QLineEdit();
  tbs_path_->setFixedWidth(kPathWidth);
  tbs_path_->setReadOnly(true);
  tbs_path_->setFocusPolicy(Qt::NoFocus);

  browse_btn_ = new QPushButton("Browse");
  load_btn_ = new QPushButton("Load");
  write_btn_ = new QPushButton("Write");

  browse_btn_->setEnabled(true);
  load_btn_->setEnabled(false);
  write_btn_->setEnabled(false);

  // Shutdown button
  power_btn_ = new PowerButton(kPowerButtonRadius);

  // Layout
  const auto pkg_btn_cols = new QHBoxLayout();
  pkg_btn_cols->addWidget(browse_btn_);
  pkg_btn_cols->addWidget(load_btn_);
  pkg_btn_cols->addWidget(write_btn_);

  const auto pkg_rows = new QVBoxLayout();
  pkg_rows->addWidget(tbs_path_);
  pkg_rows->addLayout(pkg_btn_cols);

  const auto header_cols = new QHBoxLayout();
  header_cols->addWidget(homepage_btn);
  header_cols->addWidget(urdf_builder_btn);
  header_cols->addWidget(setup_assistant_btn);
  header_cols->addWidget(hardware_setup_btn);
  header_cols->addWidget(control_system_btn);
  header_cols->addWidget(param_tuning_btn);
  header_cols->addWidget(flight_log_btn);
  header_cols->addWidget(simulation_btn);
  header_cols->addStretch();
  header_cols->addLayout(pkg_rows);
  header_cols->addSpacing(30);
  header_cols->addWidget(power_btn_);

  const auto rows = new QVBoxLayout();
  rows->addLayout(header_cols);
  rows->addWidget(app_sw);

  setLayout(rows);

  // Connection
  connect(btn_group, &QButtonGroup::idClicked, app_sw, &QStackedWidget::setCurrentIndex);
  connect(browse_btn_, &QPushButton::clicked, this, &self::onBrowseButtonClicked);
  connect(load_btn_, &QPushButton::clicked, this, &self::onLoadButtonClicked);
  connect(write_btn_, &QPushButton::clicked, this, &self::onWriteButtonClicked);
  connect(power_btn_, &QPushButton::clicked, this, &self::onShutdownButtonClicked);
}

void GUICoreWidget::updateInternalDataStructures()
{
  hardware_setup_->updateInternalDataStructures();
  control_system_->updateInternalDataStructures();
  param_tuning_->updateTBSPath(tbsPath());
  flight_log_->updateNamespace(drone_.name);
  simulation_->updateTBSPath(tbsPath());

  arming_ = nullptr;
  arming_sub_ = ros2::createSubscriber(
    node_, path::join(drone_.name, tobas::kRemoteIfaceTopicNS, tobas::kArmingTopic), &self::armingCb, this);
}

void GUICoreWidget::armingCb(const std_msgs::msg::Bool::ConstSharedPtr& arming)
{
  arming_ = arming;
}

fs::path GUICoreWidget::tbsPath() const
{
  return tbs_path_->text().toStdString();
}

void GUICoreWidget::onBrowseButtonClicked()
{
  // 前回開いたパスを取得
  std::string last_opened_dir;
  if (property_client_.get(kLastOpenedDirKey, last_opened_dir) < 0)
  {
    RCLCPP_WARN_STREAM(node_->get_logger(), property_client_.errorMessage());
    last_opened_dir = rcutils_get_home_dir();
  }

  // Tobasパッケージのパスを取得
  const auto options = QFileDialog::DontUseNativeDialog | QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks;
  const auto tbs_path =
    QFileDialog::getExistingDirectory(this, kTitle, QString::fromStdString(last_opened_dir), options);

  // キャンセルの場合は何もせずに終了 (そうしないと空文字が設定されてしまう)
  if (tbs_path.isEmpty())
    return;

  // 拡張子をチェック
  if (!tbs_path.endsWith(common::kTBSExtension))
  {
    qt::qErrorBox(this, "\"" + tbs_path + "\" is not a Tobas configuration package (*" + common::kTBSExtension + ").");
    return;
  }

  // パスをテキストに設定
  tbs_path_->setText(tbs_path);

  // ユーザが開いたディレクトリを保存
  const auto par_dir = fs::path(tbs_path.toStdString()).parent_path();
  if (property_client_.set(kLastOpenedDirKey, par_dir) < 0)
    RCLCPP_WARN_STREAM(node_->get_logger(), property_client_.errorMessage());
  if (property_client_.save() < 0)
    RCLCPP_WARN_STREAM(node_->get_logger(), property_client_.errorMessage());

  // Load,Writeボタンを有効化
  load_btn_->setEnabled(true);
  write_btn_->setEnabled(true);
}

void GUICoreWidget::onLoadButtonClicked()
{
  const auto tbs_path = tbs_path_->text().toStdString();

  // 機体設定ファイルの存在を確認
  const auto tbsdrn_path = common::getTBSDRNPath(tbs_path);
  if (!fs::is_regular_file(tbsdrn_path))
  {
    qt::qErrorBox(
      this, "\"" + QString::fromStdString(tbsdrn_path)
              + "\" does not exist. Please create a new Tobas configuration package.");
    return;
  }

  // kdl::Treeをロード
  const auto urdf_path = common::getOriginalURDFPath(tbs_path);
  if (!kdl::treeFromFile(urdf_path, tree_))
  {
    qt::qErrorBox(this, "Failed to load robot tree.");
    return;
  }

  // 機体設定ファイルをロード
  if (!drone_.load(tbsdrn_path))
  {
    qt::qErrorBox(this, "Failed to load drone configurations.");
    return;
  }

  // 内部状態を更新
  updateInternalDataStructures();

  // ロードが成功したことを示すダイアログ
  qt::qInfoBox(this, "Tobas configuration package is loaded successfully.");
}

void GUICoreWidget::onWriteButtonClicked()
{
  // アームされていないことを確認
  if (arming_ == nullptr)
  {
    if (!qt::yesOrNo(
          this,
          "This operation will restart the flight control software, "
          "so it can only be performed when the aircraft is completely stationary. "
          "Do you want to proceed?",
          qt::QMessageLevel::WARN))
      return;
  }
  else
  {
    if (arming_->data)
    {
      qt::qWarnBox(this, "This operation cannot be performed while the rotors are armed.");
      return;
    }
  }

  const auto tbs_path = tbsPath();
  const auto remote_tbs_path = common::getRemoteTBSPath(tbs_path);

  // 進捗バーを作成
  qt::ProgressDialog progress(kTitle, 7, this);
  progress.setCancelButton(nullptr);
  progress.show();

  // SSH接続
  progress.setLabelText("Connecting to the flight controller.");
  if (ssh_client_.connect() != ssh::SSHClient::E_NO_ERROR)
  {
    progress.close();
    qt::qErrorBox(this, "No SSH connection: " + QString(ssh_client_.errorMessage()));
    return;
  }
  progress.progressStep();

  // サービスを停止
  progress.setLabelText("Stopping Tobas real service.");
  if (ssh_client_.execute("systemctl stop tobas_real.target", true) != ssh::SSHClient::E_NO_ERROR)
  {
    progress.close();
    qt::qErrorBox(this, "Failed to stop Tobas real service:\n\n" + QString(ssh_client_.errorMessage()));
    return;
  }
  progress.progressStep();

  // Tobasパッケージを送信
  progress.setLabelText("Sending Tobas configuration package to the flight controller.");
  const auto mesh_path = common::getMeshPath(tbs_path);
  const auto remote_dir = fs::path(common::kColconWSPathRemote) / "src/";
  if (ssh_client_.scpPut(tbs_path, remote_dir, { mesh_path }, true) != ssh::SSHClient::E_NO_ERROR)
  {
    progress.close();
    qt::qErrorBox(this, "Failed to send Tobas configuration package:\n\n" + QString(ssh_client_.errorMessage()));
    return;
  }
  progress.progressStep();

  // Tobasパッケージをビルド
  progress.setLabelText("Building Tobas configuration package.");
  if (!package_builder_.build(remote_tbs_path))
  {
    progress.close();
    qt::qErrorBox(
      this, "Failed to build the Tobas configuration package:\n\n"
              + QString::fromStdString(package_builder_.getErrorMessage()));
    return;
  }
  progress.progressStep();

  // 環境変数TOBAS_CONFIG_PKGを設定
  progress.setLabelText("Setting environment variables.");
  const auto config_pkg_name = common::getTBSConfigName(tbs_path);
  const auto env_content = std::format("TOBAS_CONFIG_PKG={}\n", config_pkg_name);
  if (ssh_client_.sftpWrite("/etc/tobas/config_pkg.env", env_content, true) != ssh::SSHClient::E_NO_ERROR)
  {
    progress.close();
    qt::qErrorBox(this, "Failed to set environment variables:\n\n" + QString(ssh_client_.errorMessage()));
    return;
  }
  progress.progressStep();

  // サービスを再起動
  progress.setLabelText("Restarting the flight controller.");
  if (ssh_client_.execute("systemctl restart tobas_real.target", true) != ssh::SSHClient::E_NO_ERROR)
  {
    progress.close();
    qt::qErrorBox(this, "Failed to restart Tobas real service:\n\n" + QString(ssh_client_.errorMessage()));
    return;
  }
  progress.progressStep();

  // リロード
  progress.setLabelText("Reloading.");
  updateInternalDataStructures();
  progress.progressStep();

  progress.close();
  qt::qInfoBox(this, "Tobas configuration package is installed successfully.");
}

void GUICoreWidget::onShutdownButtonClicked()
{
  // アームされていないことを確認
  if (arming_ != nullptr && arming_->data)
  {
    qt::qWarnBox(this, "This operation cannot be performed while the rotors are armed.");
    return;
  }

  // SSH接続を確認
  if (ssh_client_.connect() != ssh::SSHClient::E_NO_ERROR)
  {
    qt::qErrorBox(this, "No SSH connection: " + QString(ssh_client_.errorMessage()));
    return;
  }

  // 本当にシャットダウンしてよいか確認
  if (!qt::yesOrNo(this, "Are you sure you want to shut down the FC and the GCS?", qt::QMessageLevel::WARN))
    return;

  // ラズパイをシャットダウン
  RCLCPP_INFO(node_->get_logger(), "Shutting down the flight controller.");
  ssh_client_.execute("poweroff", true, true);

  // GCSを強制終了
  rclcpp::shutdown();
  close();
  QCoreApplication::quit();
  kill(getpid(), SIGINT);
}
}  // namespace core
}  // namespace gui
