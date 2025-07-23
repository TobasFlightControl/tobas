#pragma once

#include <tobas_kdl/tree.hpp>
#include <tobas_kdl/tree_joint_axis_solver.hpp>
#include <tobas_kdl/tree_joint_parser.hpp>
#include <tobas_kdl_parser/kdl_parser.hpp>
#include <tobas_property_client/property_client.hpp>
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

namespace gui
{
namespace sa
{
class SetupAssistantWidget : public QWidget
{
  Q_OBJECT

  using self = SetupAssistantWidget;
  using super = QWidget;

  static constexpr char kLastOpenedDirKey_New[] = "last_opened_dir/setup_assistant/new";
  static constexpr char kLastOpenedDirKey_Load[] = "last_opened_dir/setup_assistant/load";
  static constexpr char kLastOpenedDirKey_Save[] = "last_opened_dir/setup_assistant/save";

  static constexpr double kJntAxisCollinearTol = tobas_std::deg2rad(5);

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

  Signals sig_;
  RotorMarkerPublisher rotor_marker_publisher_;

  QLineEdit* tbs_path_;
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

  void reset();

  void enableSaveButtons(bool enable);
  bool resolveMeshPaths(const std::filesystem::path& config_pkg_path, tinyxml2::XMLElement* elem);

  bool loadFromXml(const tinyxml2::XMLDocument* uadf_doc);
  bool loadFromText(const std::string& uadf_text);

  bool updateInternalDataStructures();

  FrameType determineFrameType();

  /* 指定したリンクの関節軸が，一般化座標に依らず指定した軸と平行であるかどうかを調べる． */
  bool isJntAxisAlwaysCollinear(const std::string& link_name, const kdl::Vector& tar_axis);

  /* 全てのスラストジョイントの関節軸が，一般化座標に依らず指定した軸と平行であるかどうかを調べる． */
  bool allThrustJointAxesAlwaysCollinear(const kdl::Vector& tar_axis);

  /* 全てのチルト軸とロータ軸が直行するかどうかを調べる． */
  bool allTiltRotorAxesPerpendicular();

private Q_SLOTS:
  void onNewButtonClicked();
  void onLoadButtonClicked();
  void onSaveButtonClicked();
  void onSaveAsButtonClicked();
};
}  // namespace sa
}  // namespace gui
