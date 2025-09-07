#pragma once

#include <tobas_qt_tools/widgets/vertical_tab_widget.hpp>

#include "./accel_calibration/widget.hpp"
#include "./mag_calibration/widget.hpp"
#include "./rcin_calibration/widget.hpp"

namespace gui
{
namespace sc
{
class SensorCalibrationWidget : public QWidget
{
  Q_OBJECT

  using self = SensorCalibrationWidget;
  using super = QWidget;

  static constexpr int kTabHeight = 35;  // これ以上無いと何故かTabBarの文字が横に見切れてしまう
  static constexpr int kTabWidth = 70;

public:
  explicit SensorCalibrationWidget(rclcpp::Node::SharedPtr node, const RosQtBridge& bridge, const tobas::Drone& drone);

  void reset();
  void updateInternalDataStructures();

private:
  const tobas::Drone& drone_;

  qt::VerticalTabWidget* tabs_;

  AccelCalibrationWidget* accel_calib_;
  MagCalibrationWidget* mag_calib_;
  RCInputCalibrationWidget* rcin_calib_;

  BaseWidget* getWidget(int index);
  const BaseWidget* getWidget(int index) const;

  void setTabsEnabled(bool enabled);

  void setCompleted(int index);
  void setIncompleted(int index);
};
}  // namespace sc
}  // namespace gui
