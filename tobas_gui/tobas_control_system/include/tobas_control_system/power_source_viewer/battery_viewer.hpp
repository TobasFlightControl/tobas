#pragma once

#include <tobas_drone_core/drone.hpp>
#include <tobas_drone_core/propulsion_system/electric_propulsion_system/electric_propulsion_system.hpp>
#include <tobas_qt_tools/widgets/position_bar_widget.hpp>
#include <tobas_rqt_bridge/bridge.hpp>

#include <tobas_msgs/msg/battery.hpp>

namespace gui
{
namespace gcs
{
class BatteryViewerWidget : public QWidget
{
  Q_OBJECT

  using self = BatteryViewerWidget;
  using super = QWidget;

  static constexpr int kLabelPSize = 12;
  static constexpr int kBarHeight = 40;

public:
  explicit BatteryViewerWidget(const RosQtBridge& bridge, const tobas::Drone& drone);

  void reset();
  void updateInternalDataStructures();

private:
  const tobas::Drone& drone_;
  tobas::ElectricPropulsionSystemConfig::ConstSharedPtr eprop_;

  qt::HPositionBarWidget* voltage_;
  qt::HPositionBarWidget* current_;

  void updateVoltage(const double& voltage);
  void updateCurrent(const double& current);

private Q_SLOTS:
  void batteryCb(const tobas_msgs::msg::Battery::ConstSharedPtr& battery);
};
}  // namespace gcs
}  // namespace gui
