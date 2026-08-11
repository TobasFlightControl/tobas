// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_qt_tools/widgets/check_box.hpp>
#include <tobas_qt_tools/widgets/spin_box.hpp>

#include "./base_setting.hpp"

namespace tobas
{
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
  static constexpr size_t kBatteryVoltageIdx = kRtComplianceIdx + 1;
  static constexpr size_t kCpuTempIdx = kBatteryVoltageIdx + 1;
  static constexpr size_t kRadioLinkIdx = kCpuTempIdx + 1;
  static constexpr size_t kRotorLinksIdx = kRadioLinkIdx + 1;
  static constexpr size_t kAttiLevelIdx = kRotorLinksIdx + 1;
  static constexpr size_t kPosStabilityIdx = kAttiLevelIdx + 1;
  static constexpr size_t kHorPosAccuracyIdx = kPosStabilityIdx + 1;
  static constexpr size_t kVerPosAccuracyIdx = kHorPosAccuracyIdx + 1;
  static constexpr size_t kAttiAccuracyIdx = kVerPosAccuracyIdx + 1;
  static constexpr size_t kHeadAccuracyIdx = kAttiAccuracyIdx + 1;
  static constexpr size_t kMagOffsetIdx = kHeadAccuracyIdx + 1;
  static constexpr size_t kMagAlignmentIdx = kMagOffsetIdx + 1;
  static constexpr size_t kVibrationLevelIdx = kMagAlignmentIdx + 1;
  static constexpr size_t kUserDefinedConditionIdx = kVibrationLevelIdx + 1;
  static constexpr size_t kItemSize = kUserDefinedConditionIdx + 1;

  static constexpr char kEscNoCommTimeoutKey[] = "esc_no_comm_timeout";

public:
  explicit FailsafeWidget();

  const char* name() const override;
  const char* title() const override;
  const char* description() const override;

  void updateInternalDataStructures() override;
  void setToDefaults() override;
  bool isValid() override;

  YAML::Node dump() const override;
  void load(const YAML::Node& node) override;

  bool checkRealtimeCompliance() const;
  bool checkBatteryVoltage() const;
  bool checkCpuTemperature() const;
  bool checkRadioLink() const;
  bool checkRotorLinks() const;
  bool checkAttitudeLevel() const;
  bool checkPositionStability() const;
  bool checkHorizontalPositionAccuracy() const;
  bool checkVerticalPositionAccuracy() const;
  bool checkAttitudeAccuracy() const;
  bool checkHeadingAccuracy() const;
  bool checkMagOffset() const;
  bool checkMagAlignment() const;
  bool checkVibrationLevel() const;
  bool checkUserDefinedCondition() const;

  double escNoCommunicationTimeout() const;  // [s]

private:
  std::array<qt::CheckBox*, kItemSize> items_;

  qt::SpinBox* esc_no_comm_timeout_;
};
}  // namespace sa
}  // namespace gui
}  // namespace tobas
