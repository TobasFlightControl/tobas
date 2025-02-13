#pragma once

#include <QWidget>

#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/string_property.hpp>

#include <tobas_qt_tools/rviz.hpp>

#include "./robot_info.hpp"

namespace gui
{
namespace sa
{
class RvizWidget : public QWidget
{
  Q_OBJECT

  using self = RvizWidget;
  using super = QWidget;

  static constexpr bool kDefaultVisualEnabled = true;
  static constexpr bool kDefaultCollisionEnabled = false;
  static constexpr bool kDefaultInertiaEnabled = false;

public:
  explicit RvizWidget(const RobotInfo& robot);

  void heightLink(const QString& link_name);
  void unheightLink(const QString& link_name);

  void resetTime();

private Q_SLOTS:
  void onRobotLoaded();
  void onVisualBoxToggled(bool checked);
  void onCollisionBoxToggled(bool checked);
  void onInertiaBoxToggled(bool checked);

private:
  const RobotInfo& robot_;

  qt::RvizFrameManager rviz_manager_;
  rviz_common::Display* display_;

  rviz_common::properties::BoolProperty* enable_visual_;
  rviz_common::properties::BoolProperty* enable_collision_;
  rviz_common::properties::BoolProperty* enable_inertia_;
  rviz_common::properties::StringProperty* highlight_link_;
  rviz_common::properties::StringProperty* unhighlight_link_;
  rviz_common::properties::BoolProperty* reload_;

  QString highlighted_link_;
};
}  // namespace sa
}  // namespace gui
