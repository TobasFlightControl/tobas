#pragma once

#include <tobas_qt_tools/widgets/double_spin_box.hpp>

#include "./base.hpp"

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
  static constexpr char kWaypointMaxVerticalVelocityParam[] = "waypoint/max_vertical_velocity";
  static constexpr char kWaypointMaxHorizontalAccelParam[] = "waypoint/max_horizontal_accel";
  static constexpr char kWaypointMaxVerticalAccelParam[] = "waypoint/max_vertical_accel";
  static constexpr char kWaypointMaxHorizontalJerkParam[] = "waypoint/max_horizontal_jerk";
  static constexpr char kWaypointMaxVerticalJerkParam[] = "waypoint/max_vertical_jerk";

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
  qt::DoubleSpinBox* wp_max_hor_vel_;
  qt::DoubleSpinBox* wp_max_ver_vel_;
  qt::DoubleSpinBox* wp_max_hor_acc_;
  qt::DoubleSpinBox* wp_max_ver_acc_;
  qt::DoubleSpinBox* wp_max_hor_jerk_;
  qt::DoubleSpinBox* wp_max_ver_jerk_;

  qt::DoubleSpinBox* takeoff_max_speed_;
  qt::DoubleSpinBox* takeoff_max_accel_;
  qt::DoubleSpinBox* takeoff_max_jerk_;

  qt::DoubleSpinBox* land_speed_;

  qt::DoubleSpinBox* rtl_min_alt_;
};
}  // namespace mission
}  // namespace sa
}  // namespace gui
