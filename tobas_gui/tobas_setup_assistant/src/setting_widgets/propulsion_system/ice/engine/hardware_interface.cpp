#include "tobas_setup_assistant/setting_tabs/propulsion_system/ice/engine/hardware_interface.hpp"

#include <tobas_yaml_tools/convert/qstring.hpp>

namespace gui
{
namespace sa
{
namespace propulsion
{
namespace ice
{
EngineHardwareIfaceWidget::EngineHardwareIfaceWidget()
{
  const auto rows = new QVBoxLayout();
  setLayout(rows);

  pwm_channel_ = new ParamGetterWidget_SpinBox("PWM Channel", "");  // TODO
  pwm_channel_->setMinimum(0);
  pwm_channel_->setValue(0);
  rows->addWidget(pwm_channel_);

  pwm_period_zero_ = new ParamGetterWidget_SpinBox("PWM Period at Zero Throttle", "");  // TODO
  pwm_period_zero_->setMinimum(0);
  pwm_period_zero_->setValue(1000);
  pwm_period_zero_->setSuffix(" us");
  rows->addWidget(pwm_period_zero_);

  pwm_period_full_ = new ParamGetterWidget_SpinBox("PWM Period at Full Throttle", "");  // TODO
  pwm_period_full_->setMinimum(0);
  pwm_period_full_->setValue(2000);
  pwm_period_full_->setSuffix(" us");
  rows->addWidget(pwm_period_full_);

  rows->addStretch();
}

bool EngineHardwareIfaceWidget::isValid()
{
  return true;
}

YAML::Node EngineHardwareIfaceWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[pwm_channel_->name()] = pwm_channel_->getValue();
  node[pwm_period_zero_->name()] = pwm_period_zero_->getValue();
  node[pwm_period_full_->name()] = pwm_period_full_->getValue();

  return node;
}

void EngineHardwareIfaceWidget::load(const YAML::Node& node)
{
  pwm_channel_->setValue(node[pwm_channel_->name()].as<int>());
  pwm_period_zero_->setValue(node[pwm_period_zero_->name()].as<int>());
  pwm_period_full_->setValue(node[pwm_period_full_->name()].as<int>());
}

int EngineHardwareIfaceWidget::pwmChannel() const
{
  return pwm_channel_->getValue();
}

int EngineHardwareIfaceWidget::pwmPeriodZeroThrot() const
{
  return pwm_period_zero_->getValue();
}

int EngineHardwareIfaceWidget::pwmPeriodFullThrot() const
{
  return pwm_period_full_->getValue();
}
}  // namespace ice
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
