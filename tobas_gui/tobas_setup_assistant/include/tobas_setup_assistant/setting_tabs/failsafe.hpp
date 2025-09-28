#pragma once

#include <QCheckBox>

#include "./base_setting.hpp"

namespace gui
{
namespace sa
{
class FailsafeWidget : public BaseSettingWidget
{
  Q_OBJECT

  using self = FailsafeWidget;
  using super = BaseSettingWidget;

  static constexpr size_t kRtComplianceIdx = 0;
  static constexpr size_t kBatteryVoltageIdx = 1;
  static constexpr size_t kCpuTempIdx = 2;
  static constexpr size_t kRotorCommIdx = 3;
  static constexpr size_t kAttiLevelIdx = 4;
  static constexpr size_t kPosStabilityIdx = 5;
  static constexpr size_t kPosAccuracyIdx = 6;
  static constexpr size_t kVelAccuracyIdx = 7;
  static constexpr size_t kAttiAccuracyIdx = 8;
  static constexpr size_t kHeadAccuracyIdx = 9;
  static constexpr size_t kMagOffsetIdx = 10;
  static constexpr size_t kMagAlignmentIdx = 11;
  static constexpr size_t kVibrationLevelIdx = 12;
  static constexpr size_t kItemSize = 13;

public:
  explicit FailsafeWidget();

  const char* name() const override;
  const char* title() const override;
  const char* description() const override;

  void updateInternalDataStructures() override;
  bool isValid() override;

  YAML::Node dump() const override;
  void load(const YAML::Node& node) override;

  bool checkRealtimeCompliance() const;
  bool checkBatteryVoltage() const;
  bool checkCpuTemperature() const;
  bool checkRotorCommunication() const;
  bool checkAttitudeLevel() const;
  bool checkPositionStability() const;
  bool checkPositionAccuracy() const;
  bool checkVelocityAccuracy() const;
  bool checkAttitudeAccuracy() const;
  bool checkHeadingAccuracy() const;
  bool checkMagOffset() const;
  bool checkMagAlignment() const;
  bool checkVibrationLevel() const;

private:
  std::array<QCheckBox*, kItemSize> items_;
};
}  // namespace sa
}  // namespace gui
