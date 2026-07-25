// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/setup_assistant.hpp"

#include <QDebug>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <tobas_constants/path.hpp>
#include <tobas_gui_common/colcon.hpp>
#include <tobas_gui_common/constants.hpp>
#include <tobas_gui_common/load_project_dialog.hpp>
#include <tobas_gui_common/project_paths.hpp>
#include <tobas_gui_common/version.hpp>
#include <tobas_path_tools/core.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/path.hpp>
#include <tobas_ros2_tools/package.hpp>
#include <tobas_ros2_tools/util.hpp>
#include <tobas_std_tools/check.hpp>
#include <tobas_string_tools/core.hpp>
#include <tobas_string_tools/stream.hpp>
#include <tobas_urdf/exporter.hpp>
#include <tobas_xml_tools/core.hpp>
#include <tobas_yaml_tools/core.hpp>

#include "tobas_setup_assistant/save_project_dialog.hpp"
#include "tobas_setup_assistant/xacro_parser.hpp"

namespace fs = std::filesystem;

namespace tobas
{
namespace gui
{
namespace sa
{
SetupAssistantWidget::SetupAssistantWidget(rclcpp::Node::SharedPtr node)
  : frame_type_detector_(uadf_, tree_)
  , property_client_(node, "tobas_setup_assistant/setup_assistant")
  , rsp_client_(node, "robot_state_publisher")
  , spinner_(Qt::WindowModal, this)
  , rotor_marker_publisher_(node, uadf_)
{
  // `--merge-install` is required to add the workspace install directory directly to the path.
  colcon_.setMergeInstall(true);

  // Package manager
  proj_path_ = new QLineEdit();
  proj_path_->setReadOnly(true);
  proj_path_->setFocusPolicy(Qt::NoFocus);

  new_btn_ = new QPushButton("New");
  load_btn_ = new QPushButton("Load");
  save_btn_ = new QPushButton("Save");
  save_as_btn_ = new QPushButton("Save As");

  enableSaveButtons(false);

  // Always allocate memory before passing pointers to other classes.
  // Otherwise, memory layout changes during allocation and causes a segmentation fault.
  rviz_ = new RvizWidget(uadf_, tree_);
  frame_tree_ = new FrameTreeWidget(tree_, rviz_);
  properties_ = new RobotPropertiesWidget(uadf_, tree_);
  jsp_ = new JointStatePublisherWidget(node, uadf_, tree_);
  settings_ = new SettingsWidget(node, uadf_, tree_, sig_);

  prj_gen_ = std::make_unique<ProjectGenerator>(node, uadf_, tree_, settings_, this);

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
  uadf_.clear();
  tree_.clear();

  // TODO: Return all settings to their startup state.
}

void SetupAssistantWidget::enableSaveButtons(bool enable)
{
  save_btn_->setEnabled(enable);
  save_as_btn_->setEnabled(enable);
}

bool SetupAssistantWidget::resolveMeshPaths(const fs::path& config_pkg_path, tinyxml2::XMLElement* elem)
{
  if (std::strcmp(elem->Name(), "mesh") == 0) {
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

  // Check child elements recursively.
  for (auto child = elem->FirstChildElement(); child; child = child->NextSiblingElement()) {
    if (!resolveMeshPaths(config_pkg_path, child)) {
      return false;
    }
  }

  return true;
}

bool SetupAssistantWidget::updateInternalDataStructures()
{
  // Update KDL objects.
  if (!frame_type_detector_.updateInternalDataStructures()) {
    return false;
  }

  // Update widgets.
  rotor_marker_publisher_.updateInternalDataStructures();
  settings_->updateInternalDataStructures();
  rviz_->updateInternalDataStructures();
  frame_tree_->updateInternalDataStructures();
  properties_->updateInternalDataStructures();
  jsp_->updateInternalDataStructures();

  // Update RSP parameter.
  const auto urdf_doc = urdf::exportUrdf(*uadf_.urdf);
  const auto urdf_text = xml::xmlDocumentToString(urdf_doc);
  if (rsp_client_.setParam("robot_description", urdf_text) != ros2::SyncParamClient::kNoError) {
    qWarning() << rsp_client_.errorMessage();
    qt::qErrorBox(this, "Failed to update the robot state publisher.");
    return false;
  }

  // Determine the frame type.
  const auto frame_type_detection_result = frame_type_detector_.determineFrameType();
  if (!frame_type_detection_result.warning_message.empty()) {
    qt::qWarnBox(this, QString::fromStdString(frame_type_detection_result.warning_message));
  }

  // Apply the frame type to widgets.
  properties_->setFrameType(frame_type_detection_result.frame_type);
  settings_->setFrameType(frame_type_detection_result.frame_type);

  return true;
}

void SetupAssistantWidget::onNewButtonClicked()
{
  // Get the previously opened path.
  std::string last_opened_dir;
  if (property_client_.get(kLastOpenedDirKey_New, last_opened_dir) < 0) {
    qWarning() << property_client_.errorMessage();
    last_opened_dir = fs::path(ament_index_cpp::get_package_share_directory("tobas_description")) / "urdf";
  }

  // Get the UADF path.
  const auto uadf_path = QFileDialog::getOpenFileName(
    this,
    "Select Aircraft Description",
    QString::fromStdString(last_opened_dir),
    "Aircraft Description (*.uadf)",
    nullptr,
    QFileDialog::DontUseNativeDialog);

  // Return without doing anything if canceled.
  if (uadf_path.isEmpty()) {
    return;
  }

  // Save the directory opened by the user.
  const auto par_dir = fs::path(uadf_path.toStdString()).parent_path();
  if (property_client_.set(kLastOpenedDirKey_New, par_dir) < 0) {
    qWarning() << property_client_.errorMessage();
  }
  if (property_client_.save() < 0) {
    qWarning() << property_client_.errorMessage();
  }

  // Build the package if the UADF exists in a ROS package before installation.
  const auto pkg_path = ros2::getPackagePathOf(uadf_path.toStdString());
  if (pkg_path && !ros2::isAlreadyBuiltAndInstalled(pkg_path.value())) {
    const auto pkg_name = ros2::getPackageNameOf(pkg_path.value());
    if (!pkg_name) {
      qt::qErrorBox(this, "Failed to get the ROS package name of the UADF: " + QString::fromStdString(pkg_name.error()));
      return;
    }
    const auto pkg_name_qt = QString::fromStdString(pkg_name.value());

    qInfo().nospace() << "UADF is in ROS package " << pkg_name_qt << ". Building it.";
    spinner_.start();
    const auto build_success = cmn::colconBuild(colcon_, pkg_path.value(), ros2::expandUser(kColconWSPathHome));
    spinner_.stop();

    if (!build_success) {
      const auto error_msg = QString::fromStdString(colcon_.errorMessage());
      if (error_msg.size() < cmn::kSaveLogTextSizeThresh) {
        qt::qErrorBox(this, "Failed to build \"" + pkg_name_qt + "\":\n\n" + error_msg);
      }
      else {
        const auto log_path =
          qt::writeTimestampedFile(error_msg + '\n', qt::expandUser(kGuiLogDir), "", "builderr_description_package");
        if (log_path) {
          qt::qErrorBox(
            this, "Failed to build \"" + pkg_name_qt + "\". The output has been saved to:\n" + log_path.value());
        }
        else {
          qt::qErrorBox(this, "Failed to build \"" + pkg_name_qt + "\", and also failed to save the error message.");
        }
      }
      return;
    }
  }

  // Parse XACRO.
  std::string uadf_text;
  if (!xacro_parser_.parseFromPath(uadf_path.toStdString(), uadf_text)) {
    const auto error_msg = QString::fromStdString(xacro_parser_.getOutput());
    if (error_msg.size() < cmn::kSaveLogTextSizeThresh) {
      qt::qErrorBox(this, "Failed to parse XACRO:\n\n" + error_msg);
    }
    else {
      const auto log_path =
        qt::writeTimestampedFile(error_msg + '\n', qt::expandUser(kGuiLogDir), "", "xacro_parse_error");
      if (log_path) {
        qt::qErrorBox(this, "Failed to parse XACRO. The output has been saved to:\n" + log_path.value());
      }
      else {
        qt::qErrorBox(this, "Failed to parse XACRO, and also failed to save the error message.");
      }
    }
    return;
  }

  // Load UADF.
  if (!uadf_parser_.parseFromText(uadf_text, uadf_)) {
    qt::qErrorBox(this, "Failed to parse UADF:\n\n" + QString::fromStdString(uadf_parser_.errorMessage()));
    reset();
    return;
  }

  // Load KDL tree.
  if (!tree_parser_.parseFromUrdf(*uadf_.urdf, tree_)) {
    qt::qErrorBox(
      this, "Failed to construct KDL tree from URDF:\n\n" + QString::fromStdString(tree_parser_.errorMessage()));
    reset();
    return;
  }

  // Check model validity.
  std::string error_msg;
  if (!uadf_.valid()) {
    qt::qErrorBox(this, "UADF is invalid.");  // TODO: Show a detailed error message.
    reset();
    return;
  }
  if (!tree_.isValid(error_msg)) {
    qt::qErrorBox(this, "UADF is invalid: " + QString::fromStdString(error_msg));
    reset();
    return;
  }

  // Update the internal state.
  if (!updateInternalDataStructures()) {
    reset();
    return;
  }

  // Clear the project path.
  proj_path_->clear();

  // Enable the save button.
  enableSaveButtons(true);

  qt::qInfoBox(this, "UADF has been loaded successfully. Configure the settings for each tab.");
}

void SetupAssistantWidget::onLoadButtonClicked()
{
  // Get the previously opened path.
  std::string last_opened_dir;
  if (property_client_.get(kLastOpenedDirKey_Load, last_opened_dir) < 0) {
    qWarning() << property_client_.errorMessage();
    last_opened_dir = ros2::expandUser(kColconWSPathHome) / "src";
    if (!fs::is_directory(last_opened_dir)) {
      last_opened_dir = ros2::getHomeDir();
    }
  }

  // Get the project path.
  cmn::LoadProjectDialog dialog(this, QString::fromStdString(last_opened_dir));
  if (dialog.exec() != QDialog::Accepted) {
    return;
  }
  const fs::path proj_path = dialog.selectedFiles().first().toStdString();
  const cmn::ProjectPaths proj_paths(proj_path);

  // Get the project version.
  cmn::Version version;
  if (!version.load(proj_paths.versionPath())) {
    qt::qErrorBox(this, "Failed to read the project version. Please create a new project.");
    return;
  }

  // If the version is incompatible, configure dynamic parameters to be initialized when the project is generated.
  if (version.isCompatible()) {
    prj_gen_->setClearDynamicParams(false);
  }
  else {
    if (!qt::yesOrNo(
          this,
          "The current Tobas version (" + cmn::Version::Current().toString() +
            ") is incompatible with the version used to create this project (" + version.toString() +
            "). Errors may occur during loading, "
            "and if you save the project again, the dynamic parameters will be reset to their default values. "
            "Would you like to proceed?",
          qt::QMessageLevel::WARN)) {
      return;
    }
    prj_gen_->setClearDynamicParams(true);
  }

  // Set the path text.
  proj_path_->setText(QString::fromStdString(proj_path));

  // Save the directory opened by the user.
  const auto par_dir = proj_path.parent_path();
  if (property_client_.set(kLastOpenedDirKey_Load, par_dir) < 0) {
    qWarning() << property_client_.errorMessage();
  }
  if (property_client_.save() < 0) {
    qWarning() << property_client_.errorMessage();
  }

  // Resolve mesh paths in the backup UADF so it can be parsed without building `config_pkg`.
  tinyxml2::XMLDocument uadf_doc;
  if (uadf_doc.LoadFile(proj_paths.originalUadfPath().c_str()) != tinyxml2::XML_SUCCESS) {
    qt::qErrorBox(this, "Failed to parse UADF document:\n\n" + QString(uadf_doc.ErrorStr()));
    return;
  }
  const auto robot = uadf_doc.RootElement();
  if (!resolveMeshPaths(proj_paths.cfgPkgPath(), robot)) {
    return;
  }

  // Load the backup UADF whose mesh paths are resolved.
  if (!uadf_parser_.parseFromXml(&uadf_doc, uadf_)) {
    qt::qErrorBox(this, "Failed to parse UADF:\n\n" + QString::fromStdString(uadf_parser_.errorMessage()));
    reset();
    return;
  }

  // Load KDL tree.
  if (!tree_parser_.parseFromUrdf(*uadf_.urdf, tree_)) {
    qt::qErrorBox(
      this, "Failed to construct KDL tree from URDF:\n\n" + QString::fromStdString(tree_parser_.errorMessage()));
    reset();
    return;
  }

  // Check model validity.
  std::string error_msg;
  if (!uadf_.valid()) {
    qt::qErrorBox(this, "UADF is invalid.");  // TODO: Show a detailed error message.
    reset();
    return;
  }
  if (!tree_.isValid(error_msg)) {
    qt::qErrorBox(this, "UADF is invalid: " + QString::fromStdString(error_msg));
    reset();
    return;
  }

  // Update the internal state.
  if (!updateInternalDataStructures()) {
    reset();
    return;
  }

  // Load user settings.
  const auto settings_path = proj_paths.backupSettingsPath();
  const auto node = yaml::load(settings_path);
  if (!node) {
    qt::qErrorBox(this, "The user configuration file is collapsed. Please create a new Tobas project.");
    reset();
    return;
  }

  // Apply user settings to widgets.
  // Do not reset even if this fails.
  if (settings_->load(node.value())) {
    qt::qInfoBox(this, "Tobas project has been loaded successfully.");
  }

  // Enable the save button.
  enableSaveButtons(true);
}

void SetupAssistantWidget::onSaveButtonClicked()
{
  // Check whether user settings have any problems.
  if (!settings_->isValid()) {
    return;
  }

  // Get the current project path.
  const auto cur_proj_path = proj_path_->text();

  // Use Save As if the project path is not set.
  if (cur_proj_path.isEmpty()) {
    onSaveAsButtonClicked();
    return;
  }

  // Create the project.
  if (!prj_gen_->generateProject(cur_proj_path.toStdString())) {
    return;
  }

  qt::qInfoBox(this, "Tobas project has been updated.");
}

void SetupAssistantWidget::onSaveAsButtonClicked()
{
  // Get the previously opened path.
  std::string last_opened_dir;
  if (property_client_.get(kLastOpenedDirKey_Save, last_opened_dir) < 0) {
    qWarning() << property_client_.errorMessage();
    last_opened_dir = ros2::expandUser(kColconWSPathHome) / "src";
    TOBAS_CHECK(path::createDirectories(last_opened_dir, true));
  }

  // Get the project path.
  const auto dflt_proj_name = "tobas_" + QString::fromStdString(uadf_.urdf->getName());
  SaveProjectDialog dialog(this, QString::fromStdString(last_opened_dir), dflt_proj_name);
  if (dialog.exec() != QDialog::Accepted) {
    return;
  }
  const auto proj_path = dialog.selectedFiles().first();
  TOBAS_CHECK(proj_path.endsWith(cmn::kProjectExtension));

  // Save the directory opened by the user.
  const auto par_dir = fs::path(proj_path.toStdString()).parent_path();
  if (property_client_.set(kLastOpenedDirKey_Save, par_dir) < 0) {
    qWarning() << property_client_.errorMessage();
  }
  if (property_client_.save() < 0) {
    qWarning() << property_client_.errorMessage();
  }

  // If an unloaded package path already exists, ask the user whether to replace it.
  if (proj_path != proj_path_->text() && fs::exists(proj_path.toStdString())) {
    if (!qt::yesOrNo(this, proj_path + " already exists. Do you want to replace it?", qt::WARN)) {
      return;
    }
    if (fs::remove_all(proj_path.toStdString()) == 0) {
      qt::qErrorBox(this, "Failed to remove " + proj_path);
      return;
    }
  }

  // Create the project.
  if (!prj_gen_->generateProject(proj_path.toStdString())) {
    return;
  }

  // Set the project path.
  proj_path_->setText(proj_path);

  qt::qInfoBox(this, "New Tobas project has been generated.");
}
}  // namespace sa
}  // namespace gui
}  // namespace tobas
