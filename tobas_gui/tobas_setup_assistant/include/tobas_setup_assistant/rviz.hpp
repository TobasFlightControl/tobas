#pragma once

#include <QWidget>

#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/string_property.hpp>

#include <tobas_rviz_wrapper/rviz.hpp>

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

  void updateInternalDataStructures();

  void heightLink(const QString& link_name);
  void unheightLink(const QString& link_name);

  /**
   * @brief rviz_common::VisualizationManager::resetTime()
   *
   * シミュレーション起動時など，TFの時刻が巻き戻ったときに発生するTF_OLD_DATAエラーを回避できる．
   */
  void resetTime();

private Q_SLOTS:
  void onVisualBoxToggled(bool checked);
  void onCollisionBoxToggled(bool checked);
  void onInertiaBoxToggled(bool checked);

private:
  const RobotInfo& robot_;

  rviz::RvizFrameManager rviz_manager_;
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
