// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_colcon_cpp/core.hpp>
#include <tobas_kdl/tree.hpp>
#include <tobas_kdl/tree_joint_axis_solver.hpp>
#include <tobas_kdl/tree_joint_parser.hpp>
#include <tobas_kdl_parser/kdl_parser.hpp>
#include <tobas_property_client/property_client.hpp>
#include <tobas_qt_tools/widgets/wait_spinner.hpp>
#include <tobas_ros2_tools/sync_param_client.hpp>
#include <tobas_std_tools/unit_conversions.hpp>
#include <tobas_uadf/model.hpp>
#include <tobas_uadf/parser.hpp>

#include "./constants.hpp"
#include "./frame_tree.hpp"
#include "./frame_type.hpp"
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

  static constexpr char kLastOpenedDirKey_New[] = "last_opened_dir/new";
  static constexpr char kLastOpenedDirKey_Load[] = "last_opened_dir/load";
  static constexpr char kLastOpenedDirKey_Save[] = "last_opened_dir/save";

  static constexpr double kJntAxisParallelTol = st::deg2rad(5);  // [rad]

public:
  explicit SetupAssistantWidget(rclcpp::Node::SharedPtr node);

private:
  uadf::Model uadf_;
  kdl::Tree tree_;

  kdl::JntArray q_zeros_;
  kdl::TreeJointParser jnt_parser_;
  kdl::TreeJointAxisSolver axis_solver_;

  XacroParser xacro_parser_;
  uadf::Parser uadf_parser_;
  kdl::TreeParser tree_parser_;

  ptree::PropertyClient property_client_;
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
  bool resolveMeshPaths(const std::filesystem::path& config_pkg_path, tinyxml2::XMLElement* elem);

  bool updateInternalDataStructures();

  FrameType determineFrameType();

  /* Check whether the joint axis of the specified link is parallel to the specified axis regardless of generalized coordinates. */
  bool isJntAxisAlwaysParallel(const std::string& link_name, const kdl::Vector& tar_axis, bool same_direction_only);

  /* Check whether all thrust joint axes are parallel to the specified axis regardless of generalized coordinates. */
  bool allThrustJointAxesAlwaysParallel(const kdl::Vector& tar_axis, bool same_direction_only);

  /* Check whether all tilt axes and rotor axes are orthogonal. */
  bool allTiltRotorAxesPerpendicular();

  /* Check whether all tilt joint axes are parallel to the specified axis regardless of generalized coordinates. */
  bool allTiltJointAxesAlwaysParallel(const kdl::Vector& tar_axis, bool same_direction_only);

  /* Check whether all tilt joint axes are mutually parallel. */
  bool allTiltJointAxesAlwaysParallel();

private Q_SLOTS:
  void onNewButtonClicked();
  void onLoadButtonClicked();
  void onSaveButtonClicked();
  void onSaveAsButtonClicked();
};
}  // namespace sa
}  // namespace gui
}  // namespace tobas
