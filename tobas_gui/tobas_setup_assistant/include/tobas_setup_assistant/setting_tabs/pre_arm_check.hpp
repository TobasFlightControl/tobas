#pragma once

#include <QCheckBox>

#include "./base_setting.hpp"

namespace gui
{
namespace sa
{
class PreArmCheckWidget : public BaseSettingWidget
{
  Q_OBJECT

  using self = PreArmCheckWidget;
  using super = BaseSettingWidget;

public:
  explicit PreArmCheckWidget();

  const char* name() const override;
  const char* title() const override;
  const char* description() const override;

  void onOpened() override;
  void updateInternalDataStructures() override;
  bool isValid() override;

  YAML::Node dump() override;
  void load(const YAML::Node& node) override;

  bool checkBatteryVoltage() const;
  bool checkCPUTemperature() const;
  bool checkRotorCommunication() const;
  bool checkAttitudeLevel() const;
  bool checkPositionStability() const;
  bool checkPositionAccuracy() const;
  bool checkOrientationAccuracy() const;
  bool checkVelocityAccuracy() const;

private:
  QCheckBox* battery_voltage_;
  QCheckBox* cpu_temperature_;
  QCheckBox* rotor_communication_;
  QCheckBox* attitude_level_;
  QCheckBox* position_stability_;
  QCheckBox* position_accuracy_;
  QCheckBox* orientation_accuracy_;
  QCheckBox* velocity_accuracy_;
};
}  // namespace sa
}  // namespace gui
