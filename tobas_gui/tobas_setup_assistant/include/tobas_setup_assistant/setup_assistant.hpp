#pragma once

#include <tobas_ros2_tools/sync_param_client.hpp>

#include "./constants.hpp"
#include "./frame_tree.hpp"
#include "./joint_state_publisher.hpp"
#include "./project_generator.hpp"
#include "./rotor_marker_publisher.hpp"
#include "./rviz.hpp"
#include "./settings.hpp"

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

public:
  explicit SetupAssistantWidget(rclcpp::Node::SharedPtr node);

  void reset();

private:
  RobotInfo robot_;
  RotorMarkerPublisher rotor_marker_publisher_;

  ptree::PropertyClient property_client_;
  ros2::SyncParamClient rsp_client_;

  QLineEdit* tbs_path_;
  QPushButton* new_btn_;
  QPushButton* load_btn_;
  QPushButton* save_btn_;
  QPushButton* save_as_btn_;

  RvizWidget* rviz_;
  FrameTreeWidget* frame_tree_;
  JointStatePublisherWidget* jsp_;
  SettingsWidget* settings_;

  std::unique_ptr<ProjectGenerator> prj_gen_;

  void enableSaveButtons(bool enable);
  bool resolveMeshPaths(const std::filesystem::path& config_pkg_path, tinyxml2::XMLElement* elem);

private Q_SLOTS:
  void onRobotLoaded();

  void onNewButtonClicked();
  void onLoadButtonClicked();
  void onSaveButtonClicked();
  void onSaveAsButtonClicked();
};
}  // namespace sa
}  // namespace gui
