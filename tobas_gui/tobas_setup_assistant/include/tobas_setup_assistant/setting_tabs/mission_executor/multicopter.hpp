#pragma once

#include <tobas_qt_tools/widgets/double_spin_box.hpp>
#include <tobas_qt_tools/widgets/spin_box.hpp>

#include "./base.hpp"

namespace tobas
{
namespace gui
{
namespace sa
{
namespace mission
{
class MulticopterWidget : public BaseExecutorWidget
{
  Q_OBJECT

  static constexpr char kWaypointMaxHorizontalVelocityParam[] = "waypoint/max_horizontal_velocity";
  static constexpr char kWaypointMaxHorizontalAccelParam[] = "waypoint/max_horizontal_accel";
  static constexpr char kWaypointMaxHorizontalJerkParam[] = "waypoint/max_horizontal_jerk";
  static constexpr char kWaypointMaxVerticalVelocityParam[] = "waypoint/max_vertical_velocity";
  static constexpr char kWaypointMaxVerticalAccelParam[] = "waypoint/max_vertical_accel";
  static constexpr char kWaypointMaxVerticalJerkParam[] = "waypoint/max_vertical_jerk";
  static constexpr char kWaypointMaxHeadingRateParam[] = "waypoint/max_heading_rate";
  static constexpr char kWaypointMaxHeadingAccelParam[] = "waypoint/max_heading_accel";

  static constexpr char kTakeoffMaxSpeedParam[] = "takeoff/max_speed";
  static constexpr char kTakeoffMaxAccelParam[] = "takeoff/max_accel";
  static constexpr char kTakeoffMaxJerkParam[] = "takeoff/max_jerk";

  static constexpr char kLandSpeedParam[] = "land/speed";

  static constexpr char kRtlMinAltitudeParam[] = "rtl/min_altitude";

public:
  explicit MulticopterWidget();

  QString executorPackage() const override;
  QString pluginName() const override;

  YAML::Node staticParams() const override;

  YAML::Node dump() const override;
  void load(const YAML::Node& node) override;

  bool isValid() override;

private:
  tobas::qt::DoubleSpinBox* wp_max_hor_vel_;
  tobas::qt::DoubleSpinBox* wp_max_hor_acc_;
  tobas::qt::DoubleSpinBox* wp_max_hor_jerk_;
  tobas::qt::DoubleSpinBox* wp_max_ver_vel_;
  tobas::qt::DoubleSpinBox* wp_max_ver_acc_;
  tobas::qt::DoubleSpinBox* wp_max_ver_jerk_;
  tobas::qt::SpinBox* wp_max_head_rate_;
  tobas::qt::SpinBox* wp_max_head_acc_;

  tobas::qt::DoubleSpinBox* takeoff_max_speed_;
  tobas::qt::DoubleSpinBox* takeoff_max_accel_;
  tobas::qt::DoubleSpinBox* takeoff_max_jerk_;

  tobas::qt::DoubleSpinBox* land_speed_;

  tobas::qt::DoubleSpinBox* rtl_min_alt_;
};
}  // namespace mission
}  // namespace sa
}  // namespace gui
}  // namespace tobas
