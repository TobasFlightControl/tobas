#pragma once

#include <moveit/robot_state_rviz_plugin/robot_state_display.h>

namespace tobas_rviz_plugin
{
class RobotStateDisplay : public moveit_rviz_plugin::RobotStateDisplay
{
  Q_OBJECT

  using self = RobotStateDisplay;
  using super = moveit_rviz_plugin::RobotStateDisplay;

  static constexpr float kHighlightR = 0;
  static constexpr float kHighlightG = 255;
  static constexpr float kHighlightB = 0;
  static constexpr float kHighlightA = 0.7;

public:
  explicit RobotStateDisplay();

private:
  rviz_common::properties::BoolProperty* enable_inertia_visible_;
  rviz_common::properties::StringProperty* highlight_link_;
  rviz_common::properties::StringProperty* unhighlight_link_;
  rviz_common::properties::BoolProperty* reload_;

private Q_SLOTS:
  void changedEnableInertiaVisible();
  void changedHighlightColor();
  void changedUnhighlightColor();
  void changedReload();
};
}  // namespace tobas_rviz_plugin
