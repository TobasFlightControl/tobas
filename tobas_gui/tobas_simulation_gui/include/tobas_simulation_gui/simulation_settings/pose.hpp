#pragma once

#include <array>

#include <tobas_qt_tools/widgets/double_spin_box.hpp>
#include <tobas_qt_tools/widgets/spin_box.hpp>

namespace gui
{
namespace sim
{
class PoseWidget : public QWidget
{
  Q_OBJECT

  using self = PoseWidget;
  using super = QWidget;

  static constexpr std::array<double, 3> kDefaultXYZ = { 0., 0., 0.5 };  // [m]
  static constexpr std::array<int, 3> kDefaultRPY = { 0, 0, 0 };         // [deg]

public:
  explicit PoseWidget();

  void reset();

  double x() const;      // [m]
  double y() const;      // [m]
  double z() const;      // [m]
  double roll() const;   // [rad]
  double pitch() const;  // [rad]
  double yaw() const;    // [rad]

private:
  std::array<qt::DoubleSpinBox*, 3> xyz_;  // [m]
  std::array<qt::SpinBox*, 3> rpy_;        // [deg]
};
}  // namespace sim
}  // namespace gui
