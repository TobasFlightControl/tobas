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
namespace hw
{
class PwmWidget : public qt::TableWidget
{
  Q_OBJECT

  using self = PwmWidget;
  using super = qt::TableWidget;

  static constexpr int kTargetNameCol = 0;
  static constexpr int kPeriodLbCol = kTargetNameCol + 1;
  static constexpr int kPeriodUbCol = kPeriodLbCol + 1;
  static constexpr int kNumCols = kPeriodUbCol + 1;

  static constexpr char kTargetNameLabel[] = "Target";
  static constexpr char kPeriodLbLabel[] = "PWM Period (LB)";
  static constexpr char kPeriodUbLabel[] = "PWM Period (UB)";

public:
  // Special target labels
  static constexpr char kEngineThrotLabel[] = "Engine Throttle";

  enum struct TargetType
  {
    kThrust,
    kControlSurface,
    kTiltJoint,
    kEngineThrottle,
  };

  explicit PwmWidget(const RobotInfo& robot, const Signals& sig);

  void updateInternalDataStructures();
  bool isValid();

  YAML::Node dump() const;
  void load(const YAML::Node& node);

  void setNumChannels(int num);

  QString targetName(int channel) const;
  TargetType targetType(int channel) const;

  uint16_t periodLb(int channel) const;  // [us]
  uint16_t periodUb(int channel) const;  // [us]

  bool contains(const QString& target_name) const;
  int channel(const QString& target_name) const;

private:
  const RobotInfo& robot_;

  tobas::propulsion_system_t prop_type_ = tobas::propulsion_system_t::ELECTRIC;

  qt::ComboBox* targetNameWidget(int row);
  qt::SpinBox* periodLbWidget(int row);
  qt::SpinBox* periodUbWidget(int row);

  const qt::ComboBox* targetNameWidget(int row) const;
  const qt::SpinBox* periodLbWidget(int row) const;
  const qt::SpinBox* periodUbWidget(int row) const;

  void addLastChannel();
  void removeLastChannel();

private Q_SLOTS:
  void onPropulsionTypeChanged(const tobas::propulsion_system_t& new_prop_type);
};
}  // namespace hw
}  // namespace sa
}  // namespace gui
