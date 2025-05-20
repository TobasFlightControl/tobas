#pragma once

#include <tobas_qt_tools/widgets/position_bar_widget.hpp>
#include <tobas_rqt_bridge/bridge.hpp>

namespace gui
{
namespace gcs
{
namespace rcin
{
class ThrottlesViewer : public QWidget
{
  Q_OBJECT

  using self = ThrottlesViewer;
  using super = QWidget;

  static constexpr int kLabelPSize = 12;
  static constexpr int kRangeSideShort = 50;

  static constexpr auto kLineColorEnable = Qt::red;
  static constexpr auto kLineColorDisable = Qt::darkGray;

public:
  explicit ThrottlesViewer(const RosQtBridge& bridge);

  void reset();

private:
  qt::HPositionBarWidget* roll_range_;
  qt::VPositionBarWidget* pitch_range_;
  qt::HPositionBarWidget* yaw_range_;
  qt::VPositionBarWidget* throt_range_;

private Q_SLOTS:
  void rcInputCb(const tobas_msgs::RCInput::ConstSharedPtr& rcin);
};
}  // namespace rcin
}  // namespace gcs
}  // namespace gui
