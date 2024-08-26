#pragma once

#include <rviz_common/visualization_manager.hpp>
#include <rviz_common/display.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/string_property.hpp>

#include <tobas_qt_tools/widgets/widget.hpp>

#include "./tree_information.hpp"

namespace gui
{
namespace setup_assistant
{
class SetupAssistant;

class RvizWidget : public qt::Widget
{
  Q_OBJECT

  using super = qt::Widget;

  static constexpr int kMinWidth = 300;
  static constexpr int kRobotStateDisplayIndex = 0;  // rvizファイルと合わせる必要あり
  static constexpr bool kDefaultVisualEnabled = true;
  static constexpr bool kDefaultCollisionEnabled = false;

public:
  explicit RvizWidget(SetupAssistant* main);

  void updateInternalDataStructures();

  void heightLink(const QString& link_name);
  void unheightLink(const QString& link_name);

private Q_SLOTS:
  void onVisualBoxToggled(bool checked);
  void onCollisionBoxToggled(bool checked);

private:
  SetupAssistant* main_;
  rviz_common::VisualizationManager* manager_;
  rviz_common::Display* display_;

  rviz_common::properties::BoolProperty* enable_visual_;
  rviz_common::properties::BoolProperty* enable_collision_;
  rviz_common::properties::BoolProperty* reload_;
  rviz_common::properties::StringProperty* highlight_link_;
  rviz_common::properties::StringProperty* unhighlight_link_;

  QString highlighted_link_;
};
}  // namespace setup_assistant
}  // namespace gui
