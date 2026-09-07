// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/setting_tabs/failsafe.hpp"

#include <QFormLayout>
#include <QHBoxLayout>
#include <QVBoxLayout>

#include <tobas_gui_common/constants.hpp>
#include <tobas_qt_tools/widgets/label.hpp>
#include <tobas_yaml_tools/convert/qstring.hpp>

namespace tobas
{
namespace gui
{
namespace sa
{
namespace
{
constexpr char kEscNoCommTimeoutKey[] = "esc_no_comm_timeout";
}  // namespace

FailsafeWidget::FailsafeWidget()
{
  items_[kRtComplianceIdx] = new qt::CheckBox("Check realtime compliance");
  items_[kBatteryVoltageIdx] = new qt::CheckBox("Check battery voltage");
  items_[kCpuTempIdx] = new qt::CheckBox("Check CPU temperature");
  items_[kRadioLinkIdx] = new qt::CheckBox("Check radio link");
  items_[kRotorLinksIdx] = new qt::CheckBox("Check rotor links");
  items_[kAttiLevelIdx] = new qt::CheckBox("Check attitude level");
  items_[kGnssFixIdx] = new qt::CheckBox("Check GNSS fix");
  items_[kPosStabilityIdx] = new qt::CheckBox("Check position stability");
  items_[kHorPosAccuracyIdx] = new qt::CheckBox("Check horizontal position accuracy");
  items_[kVerPosAccuracyIdx] = new qt::CheckBox("Check vertical position accuracy");
  items_[kAttiAccuracyIdx] = new qt::CheckBox("Check attitude accuracy");
  items_[kHeadAccuracyIdx] = new qt::CheckBox("Check heading accuracy");
  items_[kMagOffsetIdx] = new qt::CheckBox("Check magnetic field offset");
  items_[kMagAlignmentIdx] = new qt::CheckBox("Check magnetic field alignment");
  items_[kVibrationLevelIdx] = new qt::CheckBox("Check vibration level");
  items_[kUserDefinedConditionIdx] = new qt::CheckBox("Check user-defined condition");

  esc_no_comm_timeout_ = new qt::SpinBox();
  esc_no_comm_timeout_->setSuffix(" ms");

  // Layout
  const auto checklist_rows = new QVBoxLayout();
  checklist_rows->addWidget(new qt::Label("Checklist", cmn::kLabelPSize, QFont::Bold));
  for (const auto& item : items_) {
    checklist_rows->addWidget(item);
  }
  checklist_rows->addStretch();

  const auto form = new QFormLayout();
  form->addRow("ESC no communication timeout", esc_no_comm_timeout_);

  const auto detail_rows = new QVBoxLayout();
  detail_rows->addWidget(new qt::Label("Details", cmn::kLabelPSize, QFont::Bold));
  detail_rows->addLayout(form);
  detail_rows->addStretch();

  const auto cols = new QHBoxLayout();
  cols->addLayout(checklist_rows, 1);
  cols->addLayout(detail_rows, 1);

  addLayout(cols);
}

const char* FailsafeWidget::name() const
{
  return "Fail-Safe";
}

const char* FailsafeWidget::title() const
{
  return "Specify the Fail-Safe Check Items";
}

const char* FailsafeWidget::description() const
{
  return "Specify the system checks that must pass before the motors are allowed to spin. "
         "Enable a checkmark for each item to be verified. "
         "For maximum safety, enabling all checks is strongly recommended.";
}

void FailsafeWidget::updateInternalDataStructures()
{
}

void FailsafeWidget::setToDefaults()
{
  items_[kRtComplianceIdx]->setChecked(true);
  items_[kBatteryVoltageIdx]->setChecked(true);
  items_[kCpuTempIdx]->setChecked(true);
  items_[kRadioLinkIdx]->setChecked(true);
  items_[kRotorLinksIdx]->setChecked(true);
  items_[kAttiLevelIdx]->setChecked(true);
  items_[kGnssFixIdx]->setChecked(true);
  items_[kPosStabilityIdx]->setChecked(true);
  items_[kHorPosAccuracyIdx]->setChecked(true);
  items_[kVerPosAccuracyIdx]->setChecked(true);
  items_[kAttiAccuracyIdx]->setChecked(true);
  items_[kHeadAccuracyIdx]->setChecked(true);
  items_[kMagOffsetIdx]->setChecked(false);
  items_[kMagAlignmentIdx]->setChecked(false);
  items_[kVibrationLevelIdx]->setChecked(true);
  items_[kUserDefinedConditionIdx]->setChecked(false);

  esc_no_comm_timeout_->setValue(200);
}

bool FailsafeWidget::isValid()
{
  return true;
}

YAML::Node FailsafeWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  for (const auto& item : items_) {
    node[item->text()] = item->isChecked();
  }

  node[kEscNoCommTimeoutKey] = esc_no_comm_timeout_->value();

  return node;
}

void FailsafeWidget::load(const YAML::Node& node)
{
  for (const auto& item : items_) {
    item->setChecked(node[item->text()].as<bool>());
  }

  esc_no_comm_timeout_->setValue(node[kEscNoCommTimeoutKey].as<int>());
}

bool FailsafeWidget::checkRealtimeCompliance() const
{
  return items_[kRtComplianceIdx]->isChecked();
}

bool FailsafeWidget::checkBatteryVoltage() const
{
  return items_[kBatteryVoltageIdx]->isChecked();
}

bool FailsafeWidget::checkCpuTemperature() const
{
  return items_[kCpuTempIdx]->isChecked();
}

bool FailsafeWidget::checkRadioLink() const
{
  return items_[kRadioLinkIdx]->isChecked();
}

bool FailsafeWidget::checkRotorLinks() const
{
  return items_[kRotorLinksIdx]->isChecked();
}

bool FailsafeWidget::checkAttitudeLevel() const
{
  return items_[kAttiLevelIdx]->isChecked();
}

bool FailsafeWidget::checkGnssFix() const
{
  return items_[kGnssFixIdx]->isChecked();
}

bool FailsafeWidget::checkPositionStability() const
{
  return items_[kPosStabilityIdx]->isChecked();
}

bool FailsafeWidget::checkHorizontalPositionAccuracy() const
{
  return items_[kHorPosAccuracyIdx]->isChecked();
}

bool FailsafeWidget::checkVerticalPositionAccuracy() const
{
  return items_[kVerPosAccuracyIdx]->isChecked();
}

bool FailsafeWidget::checkAttitudeAccuracy() const
{
  return items_[kAttiAccuracyIdx]->isChecked();
}

bool FailsafeWidget::checkHeadingAccuracy() const
{
  return items_[kHeadAccuracyIdx]->isChecked();
}

bool FailsafeWidget::checkMagOffset() const
{
  return items_[kMagOffsetIdx]->isChecked();
}

bool FailsafeWidget::checkMagAlignment() const
{
  return items_[kMagAlignmentIdx]->isChecked();
}

bool FailsafeWidget::checkVibrationLevel() const
{
  return items_[kVibrationLevelIdx]->isChecked();
}

bool FailsafeWidget::checkUserDefinedCondition() const
{
  return items_[kUserDefinedConditionIdx]->isChecked();
}

double FailsafeWidget::escNoCommunicationTimeout() const
{
  return esc_no_comm_timeout_->value() * 1e-3;
}
}  // namespace sa
}  // namespace gui
}  // namespace tobas
