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
FailsafeWidget::FailsafeWidget()
{
  items_[kRtComplianceIdx] = new QCheckBox("Check realtime compliance");
  items_[kBatteryVoltageIdx] = new QCheckBox("Check battery voltage");
  items_[kCpuTempIdx] = new QCheckBox("Check CPU temperature");
  items_[kRadioLinkIdx] = new QCheckBox("Check radio link");
  items_[kRotorLinksIdx] = new QCheckBox("Check rotor links");
  items_[kAttiLevelIdx] = new QCheckBox("Check attitude level");
  items_[kPosStabilityIdx] = new QCheckBox("Check position stability");
  items_[kPosAccuracyIdx] = new QCheckBox("Check position accuracy");
  items_[kVelAccuracyIdx] = new QCheckBox("Check velocity accuracy");
  items_[kAttiAccuracyIdx] = new QCheckBox("Check attitude accuracy");
  items_[kHeadAccuracyIdx] = new QCheckBox("Check heading accuracy");
  items_[kMagOffsetIdx] = new QCheckBox("Check magnetic field offset");
  items_[kMagAlignmentIdx] = new QCheckBox("Check magnetic field alignment");
  items_[kVibrationLevelIdx] = new QCheckBox("Check vibration level");

  // TODO: 地磁気のオフセットを小さくできたらデフォルトをtrueにする
  for (const auto& item : items_) {
    item->setChecked(true);
  }
  items_.at(kMagOffsetIdx)->setChecked(false);
  items_.at(kMagAlignmentIdx)->setChecked(false);

  esc_no_comm_timeout_ = new tobas::qt::SpinBox();
  esc_no_comm_timeout_->setSuffix(" ms");
  esc_no_comm_timeout_->setValue(200);

  // Layout
  const auto checklist_rows = new QVBoxLayout();
  checklist_rows->addWidget(new tobas::qt::Label("Checklist", cmn::kLabelPSize, QFont::Bold));
  for (const auto& item : items_) {
    checklist_rows->addWidget(item);
  }
  checklist_rows->addStretch();

  const auto form = new QFormLayout();
  form->addRow("ESC no communication timeout", esc_no_comm_timeout_);

  const auto detail_rows = new QVBoxLayout();
  detail_rows->addWidget(new tobas::qt::Label("Details", cmn::kLabelPSize, QFont::Bold));
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

bool FailsafeWidget::checkPositionStability() const
{
  return items_[kPosStabilityIdx]->isChecked();
}

bool FailsafeWidget::checkPositionAccuracy() const
{
  return items_[kPosAccuracyIdx]->isChecked();
}

bool FailsafeWidget::checkVelocityAccuracy() const
{
  return items_[kVelAccuracyIdx]->isChecked();
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

double FailsafeWidget::escNoCommunicationTimeout() const
{
  return esc_no_comm_timeout_->value() * 1e-3;
}
}  // namespace sa
}  // namespace gui
}  // namespace tobas
