// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QSettings>

#include <tobas_colcon_cpp/core.hpp>
#include <tobas_kdl/tree.hpp>
#include <tobas_kdl_parser/kdl_parser.hpp>
#include <tobas_qt_tools/widgets/wait_spinner.hpp>
#include <tobas_ros2_tools/sync_param_client.hpp>
#include <tobas_uadf/parser.hpp>

#include "./constants.hpp"
#include "./frame_tree.hpp"
#include "./frame_type_detector.hpp"
#include "./joint_state_publisher.hpp"
#include "./project_generator.hpp"
#include "./robot_properties.hpp"
#include "./rotor_marker_publisher.hpp"
#include "./rviz.hpp"
#include "./settings.hpp"
#include "./signals.hpp"
#include "./xacro_parser.hpp"

namespace tobas
{
namespace gui
{
namespace sa
{
class SetupAssistantWidget : public QWidget
{
  Q_OBJECT

  using self = SetupAssistantWidget;
  using super = QWidget;

  static constexpr char kLastOpenedDirKey_New[] = "setup_assistant/last_opened_dir/new";
  static constexpr char kLastOpenedDirKey_Load[] = "setup_assistant/last_opened_dir/load";
  static constexpr char kLastOpenedDirKey_Save[] = "setup_assistant/last_opened_dir/save";

public:
  explicit SetupAssistantWidget(rclcpp::Node::SharedPtr node);

private:
  uadf::Model uadf_;
  kdl::Tree tree_;

  FrameTypeDetector frame_type_detector_;

  XacroParser xacro_parser_;
  uadf::Parser uadf_parser_;
  kdl::TreeParser tree_parser_;

  QSettings settings_store_;
  ros2::SyncParamClient rsp_client_;

  colcon::Colcon colcon_;

  qt::WaitSpinnerWidget spinner_;
  Signals sig_;
  RotorMarkerPublisher rotor_marker_publisher_;

  QLineEdit* proj_path_;
  QPushButton* new_btn_;
  QPushButton* load_btn_;
  QPushButton* save_btn_;
  QPushButton* save_as_btn_;

  RvizWidget* rviz_;
  FrameTreeWidget* frame_tree_;
  RobotPropertiesWidget* properties_;
  JointStatePublisherWidget* jsp_;
  SettingsWidget* settings_;

  std::unique_ptr<ProjectGenerator> prj_gen_;

  /* Return all settings to their startup state. */
  void reset();

  void enableSaveButtons(bool enable);
  bool resolveMeshPaths(const QString& config_pkg_path, tinyxml2::XMLElement* elem);

  bool updateInternalDataStructures();

private Q_SLOTS:
  void onNewButtonClicked();
  void onLoadButtonClicked();
  void onSaveButtonClicked();
  void onSaveAsButtonClicked();
};
}  // namespace sa
}  // namespace gui
}  // namespace tobas
