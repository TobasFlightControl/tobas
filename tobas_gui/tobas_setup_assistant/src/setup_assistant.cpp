#include <filesystem>

#include <tobas_qt_tools/message.hpp>

#include "tobas_setup_assistant/setup_assistant.hpp"

namespace fs = std::filesystem;

namespace gui
{
namespace sa
{
SetupAssistantWidget::SetupAssistantWidget(rclcpp::Node::SharedPtr node)
  : rotor_marker_publisher_(node, robot_, signals_),
    rsp_client_(node, "robot_state_publisher"),
    spinner_(Qt::WindowModal, this)
{
  // 他のクラスにポインタを渡す際は必ずメモリ確保してから！
  // さもないと確保時にメモリ配置が変わってセグフォになる
  settings_ = new SettingsWidget(node, robot_, signals_);
  start_ = new StartWidget(node, robot_, settings_);
  rviz_ = new RvizWidget(robot_);
  frame_tree_ = new FrameTreeWidget(robot_, rviz_);
  jsp_ = new JointStatePublisherWidget(node, robot_);

  pkg_generator_ = std::make_unique<PackageGenerator>(node, robot_, settings_);

  // Layout
  const auto cols = new QHBoxLayout();
  cols->addWidget(frame_tree_, 1);
  cols->addWidget(rviz_, 2);
  cols->addWidget(jsp_, 1);

  const auto rows = new QVBoxLayout();
  rows->addWidget(start_, 0);
  rows->addLayout(cols, 1);
  rows->addWidget(settings_, 3);

  setLayout(rows);

  // Connection
  connect(&robot_, &RobotInfo::loaded, this, &self::onRobotLoaded);
  connect(settings_->ros_package, &RosPackageWidget::generateButtonClicked, this, &self::onGenerateButtonClicked);
  connect(&build_thread_, &BuildPackageThread::finished, this, &self::onBuildPackageFinished);
}

void SetupAssistantWidget::reset()
{
  rviz_->resetTime();
}

void SetupAssistantWidget::onRobotLoaded()
{
  rotor_marker_publisher_.updateInternalDataStructures();
  settings_->updateInternalDataStructures();
  rviz_->updateInternalDataStructures();
  frame_tree_->updateInternalDataStructures();
  jsp_->updateInternalDataStructures();

  // Update RSP parameter
  if (!rsp_client_.setParam("robot_description", robot_.urdfText())) {
    qt::qErrorBox(this, "Failed to update robot state publisher.");
  }
}

void SetupAssistantWidget::onGenerateButtonClicked()
{
  // ユーザ設定に問題がないか確認
  if (!settings_->isValid()) {
    return;
  }

  // パッケージパスが既に存在する場合は置換するかどうかをユーザに確認
  const auto tbs_path = settings_->ros_package->tbsPath();
  if (fs::exists(tbs_path.toStdString())) {
    if (!qt::yesOrNo(this, tbs_path + " already exists. Do you want to replace it?", qt::QMessageLevel::WARN)) {
      return;
    }
  }

  // パッケージを作成
  if (!pkg_generator_->generatePackage()) {
    return;
  }

  // スピナーを開始
  spinner_.show();
  spinner_.start();

  // 別スレッドでTobasパッケージをビルド
  build_thread_.setPackagePath(settings_->ros_package->tbsPath());
  build_thread_.start();
}

void SetupAssistantWidget::onBuildPackageFinished(bool success, const QString& output)
{
  // スピナーを停止
  spinner_.hide();
  spinner_.stop();

  // 結果を表示
  if (success) {
    qt::qInfoBox(this, "Tobas configuration package is generated and built successfully.");
  }
  else {
    qt::qErrorBox(this, "Tobas configuration package is generated, but failed to build it:\n\n" + output);
  }
}
}  // namespace sa
}  // namespace gui
