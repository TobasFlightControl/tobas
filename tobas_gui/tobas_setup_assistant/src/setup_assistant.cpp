#include "tobas_setup_assistant/setup_assistant.hpp"

#include <filesystem>

#include <QDebug>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <tobas_gui_common/load_project_dialog.hpp>
#include <tobas_gui_common/package.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_ros2_tools/util.hpp>
#include <tobas_ros2_tools/xacro.hpp>
#include <tobas_string_tools/core.hpp>
#include <tobas_yaml_tools/core.hpp>

#include "tobas_setup_assistant/save_project_dialog.hpp"

namespace fs = std::filesystem;

namespace gui
{
namespace sa
{
SetupAssistantWidget::SetupAssistantWidget(rclcpp::Node::SharedPtr node)
  : rotor_marker_publisher_(node, robot_, signals_)
  , property_client_(node, tobas::kPropertyServerName, kPackageName)
  , rsp_client_(node, "robot_state_publisher")
{
  // Package manager
  tbs_path_ = new QLineEdit();
  tbs_path_->setReadOnly(true);
  tbs_path_->setFocusPolicy(Qt::NoFocus);

  new_btn_ = new QPushButton("New");
  load_btn_ = new QPushButton("Load");
  save_btn_ = new QPushButton("Save");
  save_as_btn_ = new QPushButton("Save As");

  enableSaveButtons(false);

  // 他のクラスにポインタを渡す際は必ずメモリ確保してから！
  // さもないと確保時にメモリ配置が変わってセグフォになる
  rviz_ = new RvizWidget(robot_);
  frame_tree_ = new FrameTreeWidget(robot_, rviz_);
  jsp_ = new JointStatePublisherWidget(node, robot_);
  settings_ = new SettingsWidget(node, robot_, signals_);

  prj_gen_ = std::make_unique<ProjectGenerator>(node, robot_, settings_);

  // Layout
  const auto pkg_cols = new QHBoxLayout();
  pkg_cols->addWidget(new_btn_);
  pkg_cols->addWidget(load_btn_);
  pkg_cols->addWidget(save_btn_);
  pkg_cols->addWidget(save_as_btn_);
  pkg_cols->addWidget(tbs_path_);

  const auto viewer_cols = new QHBoxLayout();
  viewer_cols->addWidget(frame_tree_, 1);
  viewer_cols->addWidget(rviz_, 2);
  viewer_cols->addWidget(jsp_, 1);

  const auto root_rows = new QVBoxLayout();
  root_rows->addLayout(pkg_cols, 0);
  root_rows->addLayout(viewer_cols, 2);
  root_rows->addWidget(settings_, 5);

  setLayout(root_rows);

  // Connection
  connect(&robot_, &RobotInfo::loaded, this, &self::onRobotLoaded);
  connect(new_btn_, &QPushButton::clicked, this, &self::onNewButtonClicked);
  connect(load_btn_, &QPushButton::clicked, this, &self::onLoadButtonClicked);
  connect(save_btn_, &QPushButton::clicked, this, &self::onSaveButtonClicked);
  connect(save_as_btn_, &QPushButton::clicked, this, &self::onSaveAsButtonClicked);
}

void SetupAssistantWidget::reset()
{
  rviz_->resetTime();
}

void SetupAssistantWidget::enableSaveButtons(bool enable)
{
  save_btn_->setEnabled(enable);
  save_as_btn_->setEnabled(enable);
}

bool SetupAssistantWidget::createUrdfText(const fs::path& tbs_path, std::string& text_out)
{
  // URDFの存在を確認
  const auto urdf_path = common::getProjBackupUrdfPath(tbs_path);
  if (!fs::is_regular_file(urdf_path)) {
    qt::qErrorBox(
      this, "\"" + QString::fromStdString(urdf_path) + "\" does not exist. Please create a new Tobas project.");
    return false;
  }

  // XACROを解析
  std::string urdf_text;
  if (!ros2::xacro(urdf_path, urdf_text)) {
    qt::qErrorBox(this, "Failed to convert XACRO to URDF.");
    return false;
  }

  // XMLを読み込む
  tinyxml2::XMLDocument doc;
  if (doc.Parse(urdf_text.c_str()) != tinyxml2::XML_SUCCESS) {
    qt::qErrorBox(this, "Failed to parse URDF.");
    return false;
  }
  const auto robot = doc.RootElement();

  // configパッケージが未ビルドでもメッシュパスが解析できるように絶対パスに変換
  if (!resolveMeshPaths(common::getProjCfgPkgPath(tbs_path), robot)) {
    return false;
  }

  // メッシュパス変換後のURDFを出力
  tinyxml2::XMLPrinter printer;
  doc.Print(&printer);
  text_out = printer.CStr();

  return true;
}

bool SetupAssistantWidget::resolveMeshPaths(const fs::path& config_pkg_path, tinyxml2::XMLElement* elem)
{
  if (strcmp(elem->Name(), "mesh") == 0) {
    const auto filename = elem->Attribute("filename");
    if (!filename) {
      qt::qErrorBox(settings_, "Mesh element does not have attribute: \"filename\"");
      return false;
    }

    const auto config_pkg_name = config_pkg_path.filename().string();
    const auto prefix = "package://" + config_pkg_name;

    if (std::string(filename).starts_with(prefix)) {
      const auto file_path = str::replace(std::string(filename), prefix, config_pkg_path);
      const auto new_filename = "file://" + file_path;
      elem->SetAttribute("filename", new_filename.c_str());
    }
  }

  // 再帰的に子要素もチェック
  for (auto child = elem->FirstChildElement(); child; child = child->NextSiblingElement()) {
    if (!resolveMeshPaths(config_pkg_path, child)) {
      return false;
    }
  }

  return true;
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

void SetupAssistantWidget::onNewButtonClicked()
{
  // 前回開いたパスを取得
  std::string last_opened_dir;
  if (property_client_.get(kLastOpenedDirKey_New, last_opened_dir) < 0) {
    qWarning() << property_client_.errorMessage();
    last_opened_dir = fs::path(ament_index_cpp::get_package_share_directory("tobas_description")) / "urdf";
  }

  // URDFのパスを取得
  const auto options = QFileDialog::DontUseNativeDialog;
  const auto urdf_path = QFileDialog::getOpenFileName(
    this, kTitle, QString::fromStdString(last_opened_dir), "Robot Description (*.urdf *.xacro)", nullptr, options);

  // キャンセルの場合は何もせずに終了 (そうしないと空文字が設定されてしまう)
  if (urdf_path.isEmpty()) {
    return;
  }

  // ユーザが開いたディレクトリを保存
  const auto par_dir = fs::path(urdf_path.toStdString()).parent_path();
  if (property_client_.set(kLastOpenedDirKey_New, par_dir) < 0) {
    qWarning() << property_client_.errorMessage();
  }
  if (property_client_.save() < 0) {
    qWarning() << property_client_.errorMessage();
  }

  // XACROを解析
  std::string urdf_text;
  if (!ros2::xacro(urdf_path.toStdString(), urdf_text)) {
    qt::qErrorBox(this, "Failed to convert XACRO to URDF.");
    return;
  }

  // メッシュファイルパス解析済みのURDFを作成
  if (!robot_.loadFromText(urdf_text)) {
    qt::qErrorBox(this, "Failed to load robot description.");
    return;
  }

  // プロジェクトのパスをクリア
  tbs_path_->clear();

  // 保存ボタンを有効化
  enableSaveButtons(true);

  qt::qInfoBox(this, "URDF is loaded successfully. Configure the settings for each tab.");
}

void SetupAssistantWidget::onLoadButtonClicked()
{
  // 前回開いたパスを取得
  std::string last_opened_dir;
  if (property_client_.get(kLastOpenedDirKey_Load, last_opened_dir) < 0) {
    qWarning() << property_client_.errorMessage();
    last_opened_dir = ros2::expandUser(tobas::kColconWSPathHome) / "src";
  }

  // プロジェクトのパスを取得
  common::LoadProjectDialog dialog(this, QString::fromStdString(last_opened_dir));
  if (dialog.exec() != QDialog::Accepted) {
    return;
  }
  const auto tbs_path = dialog.selectedFiles().first();
  assert(tbs_path.endsWith(tobas::kProjectExtension));

  // パスをテキストに設定
  tbs_path_->setText(tbs_path);

  // ユーザが開いたディレクトリを保存
  const auto par_dir = fs::path(tbs_path.toStdString()).parent_path();
  if (property_client_.set(kLastOpenedDirKey_Load, par_dir) < 0) {
    qWarning() << property_client_.errorMessage();
  }
  if (property_client_.save() < 0) {
    qWarning() << property_client_.errorMessage();
  }

  // config_pkgのビルドなしで解析可能なURDFを作成
  std::string urdf_text;
  if (!createUrdfText(tbs_path.toStdString(), urdf_text)) {
    return;
  }

  // URDFをロード
  // Qtのイベント処理はスタックだからこの時点で設定が各ウィジェットに反映される
  if (!robot_.loadFromText(urdf_text)) {
    qt::qErrorBox(this, "Failed to load robot description.");
    return;
  }

  // ユーザ設定を読み込む
  const auto settings_path = common::getProjBackupSettingsPath(tbs_path.toStdString());
  YAML::Node node;
  if (!yaml::load(settings_path, node)) {
    qt::qErrorBox(this, "The user configuration file is collapsed. Please create a new Tobas project.");
    return;
  }
  if (!settings_->load(node)) {
    return;
  }

  // 保存ボタンを有効化
  enableSaveButtons(true);

  qt::qInfoBox(this, "Tobas project is loaded successfully.");
}

void SetupAssistantWidget::onSaveButtonClicked()
{
  // ユーザ設定に問題がないか確認
  if (!settings_->isValid()) {
    return;
  }

  // 現在のプロジェクトパスを取得
  const auto cur_tbs_path = tbs_path_->text();

  // プロジェクトパスが設定されていない場合は名前を付けて保存
  if (cur_tbs_path.isEmpty()) {
    onSaveAsButtonClicked();
    return;
  }

  // プロジェクトを作成
  if (!prj_gen_->generateProject(cur_tbs_path.toStdString())) {
    return;
  }

  qt::qInfoBox(this, "Tobas project is updated.");
}

void SetupAssistantWidget::onSaveAsButtonClicked()
{
  // 前回開いたパスを取得
  std::string last_opened_dir;
  if (property_client_.get(kLastOpenedDirKey_Save, last_opened_dir) < 0) {
    qWarning() << property_client_.errorMessage();
    last_opened_dir = ros2::expandUser(tobas::kColconWSPathHome) / "src";
  }

  // プロジェクトのパスを取得
  SaveProjectDialog dialog(this, QString::fromStdString(last_opened_dir));
  if (dialog.exec() != QDialog::Accepted) {
    return;
  }
  const auto tbs_path = dialog.selectedFiles().first();
  assert(tbs_path.endsWith(tobas::kProjectExtension));

  // ユーザが開いたディレクトリを保存
  const auto par_dir = fs::path(tbs_path.toStdString()).parent_path();
  if (property_client_.set(kLastOpenedDirKey_Save, par_dir) < 0) {
    qWarning() << property_client_.errorMessage();
  }
  if (property_client_.save() < 0) {
    qWarning() << property_client_.errorMessage();
  }

  // 読み込んでいないパッケージパスが既に存在する場合は置換するかどうかをユーザに確認
  if (tbs_path != tbs_path_->text() && fs::exists(tbs_path.toStdString())) {
    if (!qt::yesOrNo(this, tbs_path + " already exists. Do you want to replace it?", qt::QMessageLevel::WARN)) {
      return;
    }
    if (fs::remove_all(tbs_path.toStdString()) == 0) {
      qt::qErrorBox(this, "Failed to remove " + tbs_path);
      return;
    }
  }

  // プロジェクトを作成
  if (!prj_gen_->generateProject(tbs_path.toStdString())) {
    return;
  }

  // プロジェクトのパスを設定
  tbs_path_->setText(tbs_path);

  qt::qInfoBox(this, "New Tobas project is generated.");
}
}  // namespace sa
}  // namespace gui
