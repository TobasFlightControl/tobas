#include "tobas_setup_assistant/setup_assistant.hpp"

#include <filesystem>

#include <rcutils/env.h>
#include <QDebug>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <tobas_gui_common/load_project_dialog.hpp>
#include <tobas_gui_common/project_paths.hpp>
#include <tobas_path_tools/core.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_ros2_tools/urdf_exporter.hpp>
#include <tobas_ros2_tools/util.hpp>
#include <tobas_std_tools/check.hpp>
#include <tobas_string_tools/core.hpp>
#include <tobas_string_tools/stream.hpp>
#include <tobas_xml_tools/core.hpp>
#include <tobas_yaml_tools/core.hpp>

#include "tobas_setup_assistant/save_project_dialog.hpp"
#include "tobas_setup_assistant/xacro_parser.hpp"

namespace fs = std::filesystem;

namespace gui
{
namespace sa
{
SetupAssistantWidget::SetupAssistantWidget(rclcpp::Node::SharedPtr node)
  : jnt_parser_(tree_)
  , axis_solver_(tree_)
  , property_client_(node, kPackageName)
  , rsp_client_(node, "robot_state_publisher")
  , rotor_marker_publisher_(node, uadf_)
{
  // Package manager
  proj_path_ = new QLineEdit();
  proj_path_->setReadOnly(true);
  proj_path_->setFocusPolicy(Qt::NoFocus);

  new_btn_ = new QPushButton("New");
  load_btn_ = new QPushButton("Load");
  save_btn_ = new QPushButton("Save");
  save_as_btn_ = new QPushButton("Save As");

  enableSaveButtons(false);

  // 他のクラスにポインタを渡す際は必ずメモリ確保してから！
  // さもないと確保時にメモリ配置が変わってセグフォになる
  rviz_ = new RvizWidget(uadf_, tree_);
  frame_tree_ = new FrameTreeWidget(tree_, rviz_);
  properties_ = new RobotPropertiesWidget(tree_);
  jsp_ = new JointStatePublisherWidget(node, tree_);
  settings_ = new SettingsWidget(node, uadf_, tree_, sig_);

  prj_gen_ = std::make_unique<ProjectGenerator>(node, uadf_, tree_, settings_);

  // Layout
  const auto pkg_cols = new QHBoxLayout();
  pkg_cols->addWidget(new_btn_);
  pkg_cols->addWidget(load_btn_);
  pkg_cols->addWidget(save_btn_);
  pkg_cols->addWidget(save_as_btn_);
  pkg_cols->addWidget(proj_path_);

  const auto info_rows = new QVBoxLayout();
  info_rows->addWidget(frame_tree_, 1);
  info_rows->addWidget(properties_, 1);

  const auto viewer_cols = new QHBoxLayout();
  viewer_cols->addLayout(info_rows, 1);
  viewer_cols->addWidget(rviz_, 2);
  viewer_cols->addWidget(jsp_, 1);

  const auto root_rows = new QVBoxLayout();
  root_rows->addLayout(pkg_cols, 0);
  root_rows->addLayout(viewer_cols, 2);
  root_rows->addWidget(settings_, 5);

  setLayout(root_rows);

  // Connection
  connect(new_btn_, &QPushButton::clicked, this, &self::onNewButtonClicked);
  connect(load_btn_, &QPushButton::clicked, this, &self::onLoadButtonClicked);
  connect(save_btn_, &QPushButton::clicked, this, &self::onSaveButtonClicked);
  connect(save_as_btn_, &QPushButton::clicked, this, &self::onSaveAsButtonClicked);
}

void SetupAssistantWidget::reset()
{
  // TODO: 全ての設定を起動時の状態に戻す
}

void SetupAssistantWidget::enableSaveButtons(bool enable)
{
  save_btn_->setEnabled(enable);
  save_as_btn_->setEnabled(enable);
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

bool SetupAssistantWidget::loadFromXml(const tinyxml2::XMLDocument* uadf_doc)
{
  if (!uadf_parser_.parseFromXml(uadf_doc, uadf_)) {
    qt::qErrorBox(this, "Failed to parse UADF:\n\n" + QString::fromStdString(uadf_parser_.errorMessage()));
    return false;
  }

  // Check UADF validity
  if (!uadf_.valid()) {
    qt::qErrorBox(this, "UADF is invalid.");  // TODO: 詳細なエラーメッセージを表示
    return false;
  }

  // Load KDL tree
  if (!tree_parser_.parseFromUrdf(*uadf_.urdf, tree_)) {
    qt::qErrorBox(
      this, "Failed to construct KDL tree from URDF:\n\n" + QString::fromStdString(tree_parser_.errorMessage()));
    return false;
  }

  return true;
}

bool SetupAssistantWidget::loadFromText(const std::string& uadf_text)
{
  if (!uadf_parser_.parseFromText(uadf_text, uadf_)) {
    qt::qErrorBox(this, "Failed to parse UADF:\n\n" + QString::fromStdString(uadf_parser_.errorMessage()));
    return false;
  }

  // Check UADF validity
  if (!uadf_.valid()) {
    qt::qErrorBox(this, "UADF is invalid.");  // TODO: 詳細なエラーメッセージを表示
    return false;
  }

  // Load KDL tree
  if (!tree_parser_.parseFromUrdf(*uadf_.urdf, tree_)) {
    qt::qErrorBox(
      this, "Failed to construct KDL tree from URDF:\n\n" + QString::fromStdString(tree_parser_.errorMessage()));
    return false;
  }

  return true;
}

bool SetupAssistantWidget::updateInternalDataStructures()
{
  // Update KDL objects
  q_zeros_ = kdl::JntArray::Zero(tree_.getNrOfJoints());
  if (!jnt_parser_.updateInternalDataStructures()) {
    return false;
  }
  if (!axis_solver_.updateInternalDataStructures()) {
    return false;
  }

  // Update widgets
  rotor_marker_publisher_.updateInternalDataStructures();
  settings_->updateInternalDataStructures();
  rviz_->updateInternalDataStructures();
  frame_tree_->updateInternalDataStructures();
  properties_->updateInternalDataStructures();
  jsp_->updateInternalDataStructures();

  // Update RSP parameter
  const auto urdf_doc = ros2::exportUrdf(*uadf_.urdf);
  const auto urdf_text = xml::xmlDocumentToString(urdf_doc);
  if (!rsp_client_.setParam("robot_description", urdf_text)) {
    qt::qErrorBox(this, "Failed to update robot state publisher.");
    return false;
  }

  // フレーム型を決定
  const auto frame_type = determineFrameType();
  if (frame_type == FrameType::kUndefined) {
    return false;
  }

  // フレーム型をウィジェットに反映
  properties_->setFrameType(frame_type);
  settings_->controller->setFrameType(frame_type);

  return true;
}

FrameType SetupAssistantWidget::determineFrameType()
{
  constexpr char kIsNotSupported[] = "is not supported.";

  QString msg = "Airframe\n";

  if (uadf_.control_surfaces.size() == 0) {  // 固定翼をもたない
    msg += "  - which does not have fixed wings\n";

    if (uadf_.tilts.size() == 0)  // チルトロータをもたない
    {
      msg += "  - which does not have any tilt rotors\n";

      if (uadf_.thrusts.size() < 3)  // プロペラの枚数が3枚未満
      {
        msg += "  - which has fewer than 3 propellers\n";
        qt::qWarnBox(this, msg + kIsNotSupported);
        return FrameType::kUndefined;  // TODO: 2枚なら制御可能かも
      }
      else  // プロペラの枚数が3枚以上
      {
        msg += "  - which has 3 or more propellers\n";

        if (allThrustJointAxesAlwaysParallel(kdl::Vector::UnitZ(), true))  // 全てのプロペラの回転軸が常にZ+
        {
          msg += "  - whose propeller rotation axes all point toward Z+\n";
          return FrameType::kPlanarMulticopter;  // TODO: 可操作度による分類
        }
        else  // 少なくとも1つのプロペラの回転軸がZ+以外を向く場合がある
        {
          msg += "  - which have propellers whose rotation axis can be oriented in a direction other than Z+\n";
          return FrameType::kNonPlanarMulticopter;  // TODO: 可操作度による分類
        }
      }
    }
    else  // チルトロータをもつ場合
    {
      msg += "  - which has at least one tilt rotors\n";

      if (allTiltRotorAxesPerpendicular())  // 全てのチルト軸とロータ軸が直行する
      {
        msg += "  - which has each tilt axis perpendicular to its corresponding propeller rotation axis\n";

        if (allTiltJointAxesAlwaysParallel()) {  // 全てのチルト軸が常に互いに平行
          msg += "  - whose tilt axes are all parallel to each other\n";

          if (allTiltJointAxesAlwaysParallel(kdl::Vector::UnitY(), false)) {  // 全てのチルト軸が常にY軸平行
            msg += "  - whose tilt axes are parallel to the Y axis\n";
            return FrameType::kYAxisTiltMulticopter;
          }
          else {  // 全てのチルト軸が常にY軸平行でない
            msg += "  - whose tilt axes are not parallel to the Y axis\n";
            qt::qWarnBox(this, msg + kIsNotSupported);
            return FrameType::kUndefined;
          }
        }
        else {  // 平行でないチルト軸の組が存在する
          msg += "  - there exists a pair of non-parallel tilt axes\n";
          return FrameType::kRandomAxisTiltMulticopter;
        }
      }
      else {  // チルトロータのうち，チルト軸とロータ軸が直行しないものがある
        msg += "  - which has a tilt axis that is not perpendicular to the propeller rotation axis\n";
        qt::qWarnBox(this, msg + kIsNotSupported);
        return FrameType::kUndefined;  // TODO: チルト軸と回転軸が直行しないモデルにも対応
      }
    }
  }
  else  // 固定翼をもつ場合
  {
    msg += "  - which has fixed wings\n";
    qt::qWarnBox(this, msg + kIsNotSupported);
    return FrameType::kUndefined;  // TODO: 固定翼に対応
  }
}

bool SetupAssistantWidget::isJntAxisAlwaysParallel(
  const std::string& link_name,
  const kdl::Vector& tar_axis,
  bool same_direction_only)
{
  const auto seg_it = tree_.getSegment(link_name);

  // 問題なくルートリンクまで遡れた場合はtrue．
  if (seg_it == tree_.getRootSegment()) {
    return true;
  }

  // ある関節角に対し，チェーンを構成する全てのジョイント軸が目標と平行であることが必要十分条件．
  // つまり，可動関節で且つジョイント軸が目標と平行でないリンクが存在する場合はfalse．
  const auto& joint = seg_it->second.segment.joint();
  if (joint.type != kdl::Joint::kFixed) {
    TOBAS_CHECK(axis_solver_.jntToCart(q_zeros_, link_name) == kdl::SolverI::kNoError);
    const auto& cur_axis = axis_solver_.getAxis();
    if (!cur_axis.isParallel(tar_axis, same_direction_only, kJntAxisParallelTol)) {
      return false;
    }
  }

  // 親リンクについて調べる
  const auto& par_name = seg_it->second.parent->first;
  return isJntAxisAlwaysParallel(par_name, tar_axis, same_direction_only);
}

bool SetupAssistantWidget::allThrustJointAxesAlwaysParallel(const kdl::Vector& tar_axis, bool same_direction_only)
{
  for (const auto& [joint_name, _] : uadf_.thrusts) {
    const auto& link_name = jnt_parser_.segmentName(joint_name);
    if (!isJntAxisAlwaysParallel(link_name, tar_axis, same_direction_only)) {
      return false;
    }
  }

  return true;
}

bool SetupAssistantWidget::allTiltRotorAxesPerpendicular()
{
  for (const auto& [tilt_joint_name, _] : uadf_.tilts) {
    const auto tilt_joint_urdf = uadf_.urdf->getJoint(tilt_joint_name);
    const auto& tilt_link_name = tilt_joint_urdf->child_link_name;
    const auto tilt_link_urdf = uadf_.urdf->getLink(tilt_link_name);
    const auto& thrust_joint_urdf = tilt_link_urdf->child_joints.front();
    const auto& thrust_link_name = thrust_joint_urdf->child_link_name;

    const auto& thrust_elem = tree_.getSegment(thrust_link_name)->second;
    const auto& thrust_seg = thrust_elem.segment;
    const auto& thrust_joint_kdl = thrust_seg.joint();

    const auto& tilt_elem = tree_.getSegment(tilt_link_name)->second;
    const auto& tilt_seg = tilt_elem.segment;
    const auto& tilt_joint_kdl = tilt_seg.joint();

    const auto& p = tilt_joint_kdl.axis();                         // 祖父母リンクから見たチルト軸
    const auto& q = tilt_seg.frame().M * thrust_joint_kdl.axis();  // 親リンクのジョイントフレームから見たロータ軸

    if (!p.isPerpendicular(q)) {
      return false;
    }
  }

  return true;
}

bool SetupAssistantWidget::allTiltJointAxesAlwaysParallel(const kdl::Vector& tar_axis, bool same_direction_only)
{
  for (const auto& [joint_name, _] : uadf_.tilts) {
    const auto& link_name = jnt_parser_.segmentName(joint_name);
    if (!isJntAxisAlwaysParallel(link_name, tar_axis, same_direction_only)) {
      return false;
    }
  }

  return true;
}

bool SetupAssistantWidget::allTiltJointAxesAlwaysParallel()
{
  if (uadf_.tilts.empty()) {
    qWarning() << "The drone has no tilt joints.";
    return false;
  }

  // チルト軸を1つ取得
  const auto& first_tilt_joint_name = uadf_.tilts.cbegin()->first;
  const auto& first_tilt_link_name = jnt_parser_.segmentName(first_tilt_joint_name);
  TOBAS_CHECK(axis_solver_.jntToCart(q_zeros_, first_tilt_link_name) == kdl::SolverI::kNoError);
  const auto first_tilt_joint_axis = axis_solver_.getAxis().clone();

  // 最初のチルト軸と他全てが平行ならば全てのチルト軸が互いに平行と言える
  return allTiltJointAxesAlwaysParallel(first_tilt_joint_axis, false);
}

void SetupAssistantWidget::onNewButtonClicked()
{
  // 前回開いたパスを取得
  std::string last_opened_dir;
  if (property_client_.get(kLastOpenedDirKey_New, last_opened_dir) < 0) {
    qWarning() << property_client_.errorMessage();
    last_opened_dir = fs::path(ament_index_cpp::get_package_share_directory("tobas_description")) / "urdf";
  }

  // UADFのパスを取得
  const auto options = QFileDialog::DontUseNativeDialog;
  const auto uadf_path = QFileDialog::getOpenFileName(
    this,
    "Select Aircraft Description",
    QString::fromStdString(last_opened_dir),
    "Aircraft Description (*.uadf)",
    nullptr,
    options);

  // キャンセルの場合は何もせずに終了 (そうしないと空文字が設定されてしまう)
  if (uadf_path.isEmpty()) {
    return;
  }

  // ユーザが開いたディレクトリを保存
  const auto par_dir = fs::path(uadf_path.toStdString()).parent_path();
  if (property_client_.set(kLastOpenedDirKey_New, par_dir) < 0) {
    qWarning() << property_client_.errorMessage();
  }
  if (property_client_.save() < 0) {
    qWarning() << property_client_.errorMessage();
  }

  // XACROを解析
  std::string uadf_text;
  if (!xacro_parser_.parseFromPath(uadf_path.toStdString(), uadf_text)) {
    qt::qErrorBox(this, "Failed to parse XACRO:\n\n" + QString::fromStdString(xacro_parser_.getOutput()));
    return;
  }

  // UADFを読み込む
  if (!loadFromText(uadf_text)) {
    reset();
    return;
  }

  // 内部状態を更新
  if (!updateInternalDataStructures()) {
    reset();
    return;
  }

  // プロジェクトのパスをクリア
  proj_path_->clear();

  // 保存ボタンを有効化
  enableSaveButtons(true);

  qt::qInfoBox(this, "UADF is loaded successfully. Configure the settings for each tab.");
}

void SetupAssistantWidget::onLoadButtonClicked()
{
  // 前回開いたパスを取得
  std::string last_opened_dir;
  if (property_client_.get(kLastOpenedDirKey_Load, last_opened_dir) < 0) {
    qWarning() << property_client_.errorMessage();
    last_opened_dir = ros2::expandUser(tobas::kColconWSPathHome) / "src";
    if (!fs::is_directory(last_opened_dir)) {
      last_opened_dir = rcutils_get_home_dir();
    }
  }

  // プロジェクトのパスを取得
  cmn::LoadProjectDialog dialog(this, QString::fromStdString(last_opened_dir));
  if (dialog.exec() != QDialog::Accepted) {
    return;
  }
  const fs::path proj_path = dialog.selectedFiles().first().toStdString();
  const cmn::ProjectPaths proj_paths(proj_path);

  // パスをテキストに設定
  proj_path_->setText(QString::fromStdString(proj_path));

  // ユーザが開いたディレクトリを保存
  const auto par_dir = proj_path.parent_path();
  if (property_client_.set(kLastOpenedDirKey_Load, par_dir) < 0) {
    qWarning() << property_client_.errorMessage();
  }
  if (property_client_.save() < 0) {
    qWarning() << property_client_.errorMessage();
  }

  // バックアップUADFのメッシュパスを解決 (config_pkgのビルドなしで解析可能に)
  std::string uadf_text;
  if (!str::readText(proj_paths.originalUadfPath(), uadf_text)) {
    qt::qErrorBox(this, "Failed to open file: " + QString::fromStdString(proj_path));
    return;
  }
  tinyxml2::XMLDocument uadf_doc;
  if (uadf_doc.Parse(uadf_text.c_str()) != tinyxml2::XML_SUCCESS) {
    qt::qErrorBox(this, "Failed to parse UADF document.");
    return;
  }
  const auto robot = uadf_doc.RootElement();
  if (!resolveMeshPaths(proj_paths.cfgPkgPath(), robot)) {
    return;
  }

  // メッシュパスを解決したバックアップUADFを読み込む
  if (!loadFromXml(&uadf_doc)) {
    reset();
    return;
  }

  // 内部状態を更新
  if (!updateInternalDataStructures()) {
    reset();
    return;
  }

  // ユーザ設定を読み込む
  const auto settings_path = proj_paths.backupSettingsPath();
  const auto node = yaml::load(settings_path);
  if (!node) {
    qt::qErrorBox(this, "The user configuration file is collapsed. Please create a new Tobas project.");
    reset();
    return;
  }

  // ユーザ設定を書くウィジェットに反映
  // 失敗してもリセットはしない
  if (settings_->load(node.value())) {
    qt::qInfoBox(this, "Tobas project is loaded successfully.");
  }

  // 保存ボタンを有効化
  enableSaveButtons(true);
}

void SetupAssistantWidget::onSaveButtonClicked()
{
  // ユーザ設定に問題がないか確認
  if (!settings_->isValid()) {
    return;
  }

  // 現在のプロジェクトパスを取得
  const auto cur_proj_path = proj_path_->text();

  // プロジェクトパスが設定されていない場合は名前を付けて保存
  if (cur_proj_path.isEmpty()) {
    onSaveAsButtonClicked();
    return;
  }

  // プロジェクトを作成
  if (!prj_gen_->generateProject(cur_proj_path.toStdString())) {
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
    TOBAS_CHECK(path::createDirectories(last_opened_dir, true));
  }

  // プロジェクトのパスを取得
  const auto dflt_proj_name = "tobas_" + QString::fromStdString(uadf_.urdf->getName());
  SaveProjectDialog dialog(this, QString::fromStdString(last_opened_dir), dflt_proj_name);
  if (dialog.exec() != QDialog::Accepted) {
    return;
  }
  const auto proj_path = dialog.selectedFiles().first();
  TOBAS_CHECK(proj_path.endsWith(tobas::kProjectExtension));

  // ユーザが開いたディレクトリを保存
  const auto par_dir = fs::path(proj_path.toStdString()).parent_path();
  if (property_client_.set(kLastOpenedDirKey_Save, par_dir) < 0) {
    qWarning() << property_client_.errorMessage();
  }
  if (property_client_.save() < 0) {
    qWarning() << property_client_.errorMessage();
  }

  // 読み込んでいないパッケージパスが既に存在する場合は置換するかどうかをユーザに確認
  if (proj_path != proj_path_->text() && fs::exists(proj_path.toStdString())) {
    if (!qt::yesOrNo(this, proj_path + " already exists. Do you want to replace it?", qt::QMessageLevel::WARN)) {
      return;
    }
    if (fs::remove_all(proj_path.toStdString()) == 0) {
      qt::qErrorBox(this, "Failed to remove " + proj_path);
      return;
    }
  }

  // プロジェクトを作成
  if (!prj_gen_->generateProject(proj_path.toStdString())) {
    return;
  }

  // プロジェクトのパスを設定
  proj_path_->setText(proj_path);

  qt::qInfoBox(this, "New Tobas project is generated.");
}
}  // namespace sa
}  // namespace gui
