#pragma once

#include <tobas_qt_tools/widgets/position_bar_widget.hpp>
#include <tobas_rqt_bridge/bridge.hpp>

namespace tobas
{
namespace gui
{
namespace ctrl
{
namespace rcin
{
class ThrottlesViewer : public QWidget
{
  Q_OBJECT

  using self = ThrottlesViewer;
  using super = QWidget;

  static constexpr int kLabelPSize = 12;
  static constexpr int kRangeSideShort = 30;

  static constexpr auto kLineColorEnable = Qt::red;
  static constexpr auto kLineColorDisable = Qt::darkGray;

public:
  explicit ThrottlesViewer(const RosQtBridge& bridge);

  void reset();

private:
  tobas::qt::HPositionBarWidget* roll_range_;
  tobas::qt::VPositionBarWidget* pitch_range_;
  tobas::qt::HPositionBarWidget* yaw_range_;
  tobas::qt::VPositionBarWidget* throt_range_;

private Q_SLOTS:
  void rcInputCb(const tobas_msgs::RCInput::ConstSharedPtr& rcin);
};
}  // namespace rcin
}  // namespace ctrl
}  // namespace gui
}  // namespace tobas
