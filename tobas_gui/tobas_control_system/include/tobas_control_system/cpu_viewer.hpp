#pragma once

#include <tobas_qt_tools/widgets/position_bar_widget.hpp>
#include <tobas_rqt_bridge/bridge.hpp>

namespace gui
{
namespace ctrl
{
class CpuViewerWidget : public QWidget
{
  Q_OBJECT

  using self = CpuViewerWidget;
  using super = QWidget;

  static constexpr int kLabelPSize = 12;
  static constexpr int kBarHeight = 30;

  static constexpr double kMinTemp = 0.;    // [degC]
  static constexpr double kMaxTemp = 100.;  // [degC]
  static constexpr double kMinLoad = 0.;    // [%]
  static constexpr double kMaxLoad = 100.;  // [%]

public:
  explicit CpuViewerWidget(const RosQtBridge& bridge);

  void reset();

private:
  qt::HPositionBarWidget* temp_;
  qt::HPositionBarWidget* load_;

private Q_SLOTS:
  void cpuCb(const tobas_msgs::msg::Cpu::ConstSharedPtr& cpu);
};
}  // namespace ctrl
}  // namespace gui
