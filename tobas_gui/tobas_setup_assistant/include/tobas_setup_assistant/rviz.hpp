#pragma once

#include <rviz_common/visualization_manager.hpp>
#include <rviz_common/display.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/string_property.hpp>

#include <tobas_qt_tools/widgets/widget.hpp>

#include "./robot_info.hpp"

namespace gui
{
namespace setup_assistant
{
class RvizWidget : public qt::Widget
{
  Q_OBJECT

  using self = RvizWidget;
  using super = qt::Widget;

  static constexpr int kRobotStateDisplayIndex = 0;  // rvizファイルと合わせる必要あり
  static constexpr bool kDefaultVisualEnabled = true;
  static constexpr bool kDefaultCollisionEnabled = false;

public:
  explicit RvizWidget(
    rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_node_if,
    const RobotInfo& robot);

  void heightLink(const QString& link_name);
  void unheightLink(const QString& link_name);

private Q_SLOTS:
  void onRobotLoaded(const QString& urdf_content);
  void onVisualBoxToggled(bool checked);
  void onCollisionBoxToggled(bool checked);

private:
  const RobotInfo& robot_;

  const rclcpp::Node::SharedPtr rviz_node_;

  rviz_common::VisualizationManager* manager_;
  rviz_common::Display* display_;

  rviz_common::properties::BoolProperty* enable_visual_;
  rviz_common::properties::BoolProperty* enable_collision_;
  rviz_common::properties::StringProperty* highlight_link_;
  rviz_common::properties::StringProperty* unhighlight_link_;
  rviz_common::properties::BoolProperty* reload_;

  QString highlighted_link_;
};
}  // namespace setup_assistant
}  // namespace gui
