#include <QLabel>
#include <QVBoxLayout>
#include <QHBoxLayout>

#include <tobas_path_tools/join.hpp>
#include <tobas_linux/core.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/widgets/combo_box.hpp>
#include <tobas_qt_tools/widgets/stacked_widget.hpp>
#include <tobas_qt_tools/widgets/progress_dialog.hpp>
#include <tobas_gui_common/constants.hpp>
#include <tobas_gui_common/package.hpp>

#include "tobas_gui_core/gui_core.hpp"
#include "tobas_gui_core/constants.hpp"

namespace fs = std::filesystem;

namespace gui
{
namespace core
{
GUICoreWidget::GUICoreWidget(rclcpp::Node::SharedPtr node)
  : node_(node), property_cli_(node, tobas::kPropertyServerName, kPkgName), ssh_cli_(node), package_builder_(node)
{
  // Applications
  const auto app_cb = new qt::ComboBox();
  const auto app_sw = new qt::StackedWidget();
  connect(app_cb, QOverload<int>::of(&QComboBox::currentIndexChanged), app_sw, &QStackedWidget::setCurrentIndex);

  homepage_ = new homepage::HomepageWidget();
  app_cb->addItem("Homepage");
  app_sw->addWidget(homepage_);

  urdf_builder_ = new URDFBuilder();
  app_cb->addItem("URDF Builder");
  app_sw->addWidget(urdf_builder_);

  setup_assistant_ = new setup_assistant::SetupAssistantWidget(node);
  app_cb->addItem("Setup Assistant");
  app_sw->addWidget(setup_assistant_);

  hardware_setup_ = new hardware_setup::HardwareSetupWidget(node, drone_);
  app_cb->addItem("Hardware Setup");
  app_sw->addWidget(hardware_setup_);

  control_system_ = new control_system::ControlSystemWidget(node, drone_);
  app_cb->addItem("Control System");
  app_sw->addWidget(control_system_);

  console_ = new console::ConsoleWidget(node);
  app_cb->addItem("Console Message");
  app_sw->addWidget(console_);

  param_tuning_ = new param_tuning::ParameterTuningWidget(node);
  app_cb->addItem("Parameter Tuning");
  app_sw->addWidget(param_tuning_);

  flight_log_ = new log::FlightLogWidget(node);
  app_cb->addItem("Flight Log");
  app_sw->addWidget(flight_log_);

  // Package manager
  tbs_path_ = new QLineEdit();
  tbs_path_->setFixedWidth(kPathWidth);
  tbs_path_->setReadOnly(true);
  tbs_path_->setFocusPolicy(Qt::NoFocus);

  load_button_ = new QPushButton("Load");
  load_button_->setFixedWidth(kButtonWidth);
  connect(load_button_, &QPushButton::clicked, this, &self::onLoadButtonClicked);

  send_button_ = new QPushButton("Send");
  send_button_->setFixedWidth(kButtonWidth);
  send_button_->setEnabled(false);
  connect(send_button_, &QPushButton::clicked, this, &self::onSendButtonClicked);

  // Shutdown button
  shutdown_button_ = new QPushButton("Shutdown");
  shutdown_button_->setStyleSheet("background-color: red");
  connect(shutdown_button_, &QPushButton::clicked, this, &self::onShutdownButtonClicked);

  // Header layout
  const auto header_cols = new QHBoxLayout();
  header_cols->addWidget(app_cb);
  header_cols->addStretch();
  header_cols->addWidget(tbs_path_);
  header_cols->addWidget(load_button_);
  header_cols->addWidget(send_button_);
  header_cols->addStretch();
  header_cols->addWidget(shutdown_button_);

  // Overall layout
  const auto rows = new QVBoxLayout();
  setLayout(rows);
  rows->addLayout(header_cols);
  rows->addWidget(app_sw);
}

void GUICoreWidget::updateInternalDataStructures()
{
  hardware_setup_->updateInternalDataStructures();
  control_system_->updateInternalDataStructures();
  console_->updateNamespace(drone_.name);
  param_tuning_->updateTBSPath(tbsPath());
  flight_log_->updateNamespace(drone_.name);

  arming_ = nullptr;
  arming_sub_ = ros2::createSubscriber(node_, path::join(drone_.name, tobas::kArmingTopic), &self::armingCb, this);
}

void GUICoreWidget::armingCb(const std_msgs::msg::Bool::ConstSharedPtr& arming)
{
  arming_ = arming;
}

fs::path GUICoreWidget::tbsPath() const
{
  return tbs_path_->text().toStdString();
}

void GUICoreWidget::onLoadButtonClicked()
{
  // 前回開いたパスを取得
  std::string last_opened_dir;
  if (property_cli_.get(kLastOpenedDirKey, last_opened_dir) < 0)
  {
    RCLCPP_WARN_STREAM(node_->get_logger(), property_cli_.errorMessage());
    last_opened_dir = linux::homeDir();
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

  // 機体設定ファイルの存在を確認
  const auto tbsdrn_path = common::getTBSDRNPath(tbs_path.toStdString());
  if (!fs::is_regular_file(tbsdrn_path))
  {
    qt::qErrorBox(
      this, "\"" + QString::fromStdString(tbsdrn_path)
              + "\" does not exist. Please create a new Tobas configuration package.");
    return;
  }

  // 機体設定ファイルをロード
  if (!drone_.load(tbsdrn_path))
  {
    qt::qErrorBox(this, "Failed to load drone configurations.");
    return;
  }

  // パスをテキストに設定
  tbs_path_->setText(tbs_path);

  // ユーザが開いたディレクトリを保存
  const auto par_dir = fs::path(tbs_path.toStdString()).parent_path();
  if (property_cli_.set(kLastOpenedDirKey, par_dir) < 0)
    RCLCPP_WARN_STREAM(node_->get_logger(), property_cli_.errorMessage());
  if (property_cli_.save() < 0)
    RCLCPP_WARN_STREAM(node_->get_logger(), property_cli_.errorMessage());

  // Writeボタンを有効化
  send_button_->setEnabled(true);

  // 内部状態を更新
  updateInternalDataStructures();

  // ロードが成功したことを示すダイアログ
  qt::qInfoBox(this, "Tobas configuration package is loaded successfully.");
}

void GUICoreWidget::onSendButtonClicked()
{
  // SSH接続を確認
  if (!ssh_cli_.isConnected())
  {
    qt::qWarnBox(this, "No SSH connection.");
    return;
  }

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
  qt::ProgressDialog progress(kTitle, 6, this);
  progress.setCancelButton(nullptr);
  progress.show();

  // サービスを停止
  progress.setLabelText("Stopping Tobas flight controller.");
  if (ssh_cli_.execute("systemctl stop tobas_real.service", true) != ssh::SSHClient::E_NO_ERROR)
  {
    progress.close();
    qt::qErrorBox(this, "Failed to stop Tobas:\n\n" + QString(ssh_cli_.errorMessage()));
    return;
  }
  progress.progressStep();

  // Tobasパッケージを送信
  progress.setLabelText("Sending Tobas configuration package to the flight controller.");
  const auto mesh_path = common::getMeshPath(tbs_path);
  const auto remote_dir = fs::path(common::kColconWSPathRemote) / "src/";
  if (ssh_cli_.scpPut(tbs_path, remote_dir, { mesh_path }, true) != ssh::SSHClient::E_NO_ERROR)
  {
    progress.close();
    qt::qErrorBox(this, "Failed to send tobas configuration package\n\n" + QString(ssh_cli_.errorMessage()));
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
  const auto env_content = std::format("TOBAS_CONFIG_PKG={}\nDRONE_NAME={}\n", config_pkg_name, drone_.name);
  if (ssh_cli_.sftpWrite("/etc/tobas/config_pkg.env", env_content, true) != ssh::SSHClient::E_NO_ERROR)
  {
    progress.close();
    qt::qErrorBox(this, "Failed to set environment variables:\n\n" + QString(ssh_cli_.errorMessage()));
    return;
  }
  progress.progressStep();

  // サービスを再起動
  progress.setLabelText("Restarting Tobas flight controller.");
  if (ssh_cli_.execute("systemctl restart tobas_real.service", true) != ssh::SSHClient::E_NO_ERROR)
  {
    progress.close();
    qt::qErrorBox(this, "Failed to restart Tobas:\n\n" + QString(ssh_cli_.errorMessage()));
    return;
  }
  progress.progressStep();

  // GCSをリロード
  progress.setLabelText("Reloading GCS");
  updateInternalDataStructures();
  progress.progressStep();

  progress.close();
  qt::qInfoBox(this, "Tobas configuration package is installed successfully.");
}

void GUICoreWidget::onShutdownButtonClicked()
{
  // SSH接続を確認
  if (!ssh_cli_.isConnected())
  {
    qt::qErrorBox(this, "No SSH connection.");
    return;
  }

  // アームされていないことを確認
  if (arming_ != nullptr && arming_->data)
  {
    qt::qWarnBox(this, "This operation cannot be performed while the rotors are armed.");
    return;
  }

  // 本当にシャットダウンしてよいか確認
  if (!qt::yesOrNo(this, "Are you sure you want to shut down the FC and the GCS?", qt::QMessageLevel::WARN))
    return;

  // ラズパイをシャットダウン
  RCLCPP_INFO(node_->get_logger(), "Shutting down the flight controller.");
  ssh_cli_.execute("poweroff", true, true);

  // GCSを強制終了
  kill(getpid(), SIGINT);
}
}  // namespace core
}  // namespace gui
