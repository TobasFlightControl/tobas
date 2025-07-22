#pragma once

#include <yaml-cpp/yaml.h>

#include <tobas_qt_tools/widgets/combo_box.hpp>
#include <tobas_qt_tools/widgets/spin_box.hpp>
#include <tobas_qt_tools/widgets/table_widget.hpp>
#include <tobas_uadf/model.hpp>

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

  static constexpr int kPeriodDecimals = 2;

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

  explicit PwmWidget(const uadf::Model& uadf, const Signals& sig);

  void updateInternalDataStructures();
  bool isValid();

  YAML::Node dump() const;
  void load(const YAML::Node& node);

  void setNumChannels(int num);

  QString targetName(int channel) const;
  TargetType targetType(int channel) const;

  double periodLb(int channel) const;  // [us]
  double periodUb(int channel) const;  // [us]

  bool contains(const QString& target_name) const;
  int channel(const QString& target_name) const;

private:
  const uadf::Model& uadf_;

  tobas::PropulsionSystem prop_type_ = tobas::PropulsionSystem::kElectric;

  qt::ComboBox* targetNameWidget(int row);
  qt::DoubleSpinBox* periodLbWidget(int row);
  qt::DoubleSpinBox* periodUbWidget(int row);

  const qt::ComboBox* targetNameWidget(int row) const;
  const qt::DoubleSpinBox* periodLbWidget(int row) const;
  const qt::DoubleSpinBox* periodUbWidget(int row) const;

  void addLastChannel();
  void removeLastChannel();

private Q_SLOTS:
  void onPropulsionTypeChanged(const tobas::PropulsionSystem& new_prop_type);
};
}  // namespace hw
}  // namespace sa
}  // namespace gui
