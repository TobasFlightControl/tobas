#include <QHBoxLayout>

#include "tobas_setup_assistant/setting_tabs/propulsion_system/speed_limit/base.hpp"

namespace gui
{
namespace sa
{
namespace propulsion
{
void SpeedLimitWidget_Base::initialize(QButtonGroup* ckb_group)
{
  const auto cols = new QHBoxLayout();
  setLayout(cols);

  checkbox_ = new QCheckBox(name());
  cols->addWidget(checkbox_);
  ckb_group->addButton(checkbox_);

  spinbox_ = new qt::DoubleSpinBox();
  spinbox_->setEnabled(false);
  cols->addWidget(spinbox_);

  connect(checkbox_, &QCheckBox::toggled, spinbox_, &qt::DoubleSpinBox::setEnabled);

  onInit();
}

void SpeedLimitWidget_Base::copyFrom(const SpeedLimitWidget_Base* src)
{
  checkbox_->setChecked(src->checkbox_->isChecked());
  spinbox_->setValue(src->spinbox_->value());
}

YAML::Node SpeedLimitWidget_Base::dump()
{
  YAML::Node node(YAML::NodeType::Map);

  node[kIsCheckedKey] = checkbox_->isChecked();
  node[kValueKey] = spinbox_->value();

  return node;
}

void SpeedLimitWidget_Base::load(const YAML::Node& node)
{
  checkbox_->setChecked(node[kIsCheckedKey].as<bool>());
  spinbox_->setValue(node[kValueKey].as<double>());
}

bool SpeedLimitWidget_Base::isChecked() const
{
  return checkbox_->isChecked();
}

void SpeedLimitWidget_Base::setChecked(bool checked)
{
  checkbox_->setChecked(checked);
}
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
