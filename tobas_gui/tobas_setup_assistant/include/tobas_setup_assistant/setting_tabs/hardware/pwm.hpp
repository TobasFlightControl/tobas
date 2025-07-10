#pragma once

#include <yaml-cpp/yaml.h>

#include <tobas_qt_tools/widgets/combo_box.hpp>
#include <tobas_qt_tools/widgets/spin_box.hpp>
#include <tobas_qt_tools/widgets/table_widget.hpp>

#include "tobas_setup_assistant/robot_info.hpp"
#include "tobas_setup_assistant/signals.hpp"

namespace gui
{
namespace sa
{
class PwmWidget : public qt::TableWidget
{
  Q_OBJECT

  using self = PwmWidget;
  using super = qt::TableWidget;

  static constexpr int kTargetNameCol = 0;
  static constexpr int kPwmPeriodLbCol = kTargetNameCol + 1;
  static constexpr int kPwmPeriodUbCol = kPwmPeriodLbCol + 1;
  static constexpr int kNumCols = kPwmPeriodUbCol + 1;

  static constexpr char kTargetNameLabel[] = "Target";
  static constexpr char kPwmPeriodLbLabel[] = "PWM Period (LB)";
  static constexpr char kPwmPeriodUbLabel[] = "PWM Period (UB)";

  // Special target labels
  static constexpr char kEngineThrotLabel[] = "Engine Throttle";

public:
  enum struct TargetType
  {
    kThrust,
    kTiltJoint,
    kControlSurface,
    kEngineThrottle,
  };

  explicit PwmWidget(const RobotInfo& robot, const Signals& sig);

  YAML::Node dump() const;
  void load(const YAML::Node& node);

  void setNumChannels(int num);

  QString targetName(int channel) const;
  TargetType targetType(int channel) const;

  uint16_t pwmPeriodLb(int channel) const;  // [us]
  uint16_t pwmPeriodUb(int channel) const;  // [us]

private:
  const RobotInfo& robot_;

  tobas::propulsion_system_t prop_type_ = tobas::propulsion_system_t::ELECTRIC;

  QVector<qt::ComboBox*> target_names_;
  QVector<qt::SpinBox*> periods_lb_;  // [us]
  QVector<qt::SpinBox*> periods_ub_;  // [us]

  void addLastChannel();
  void removeLastChannel();

private Q_SLOTS:
  void onPropulsionTypeChanged(const tobas::propulsion_system_t& new_prop_type);
};
}  // namespace sa
}  // namespace gui
