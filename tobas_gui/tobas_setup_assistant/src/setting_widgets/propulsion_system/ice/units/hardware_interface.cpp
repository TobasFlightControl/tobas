#include "tobas_setup_assistant/setting_tabs/propulsion_system/ice/propulsion_units/hardware_interface.hpp"

#include <tobas_yaml_tools/convert/qstring.hpp>
#include <tobas_qt_tools/cast.hpp>

namespace gui
{
namespace sa
{
namespace propulsion
{
namespace ice
{
VPitchHardwareIfaceWidget::VPitchHardwareIfaceWidget()
{
  const auto rows = new QVBoxLayout();
  setLayout(rows);

  pwm_channel_ = new ParamGetterWidget_SpinBox("PWM Channel", "");  // TODO
  pwm_channel_->setMinimum(0);
  pwm_channel_->setValue(0);
  rows->addWidget(pwm_channel_);

  pwm_period_min_pitch_ = new ParamGetterWidget_SpinBox("PWM Period at Minimum Pitch Angle", "");  // TODO
  pwm_period_min_pitch_->setMinimum(0);
  pwm_period_min_pitch_->setValue(1000);
  pwm_period_min_pitch_->setSuffix(" us");
  rows->addWidget(pwm_period_min_pitch_);

  pwm_period_max_pitch_ = new ParamGetterWidget_SpinBox("PWM Period at Maximum Pitch Angle", "");  // TODO
  pwm_period_max_pitch_->setMinimum(0);
  pwm_period_max_pitch_->setValue(2000);
  pwm_period_max_pitch_->setSuffix(" us");
  rows->addWidget(pwm_period_max_pitch_);

  rows->addStretch();
}

const char* VPitchHardwareIfaceWidget::name() const
{
  return "HW Interface";
}

bool VPitchHardwareIfaceWidget::isValid()
{
  return true;
}

void VPitchHardwareIfaceWidget::copyFrom(const BaseSelectedLinkSettingWidget* src)
{
  const auto derived = qt::qConstPointerCast<VPitchHardwareIfaceWidget>(src);

  pwm_channel_->setValue(derived->pwm_channel_->getValue());
  pwm_period_min_pitch_->setValue(derived->pwm_period_min_pitch_->getValue());
  pwm_period_max_pitch_->setValue(derived->pwm_period_max_pitch_->getValue());
}

YAML::Node VPitchHardwareIfaceWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[pwm_channel_->name()] = pwm_channel_->getValue();
  node[pwm_period_min_pitch_->name()] = pwm_period_min_pitch_->getValue();
  node[pwm_period_max_pitch_->name()] = pwm_period_max_pitch_->getValue();

  return node;
}

void VPitchHardwareIfaceWidget::load(const YAML::Node& node)
{
  pwm_channel_->setValue(node[pwm_channel_->name()].as<int>());
  pwm_period_min_pitch_->setValue(node[pwm_period_min_pitch_->name()].as<int>());
  pwm_period_max_pitch_->setValue(node[pwm_period_max_pitch_->name()].as<int>());
}

int VPitchHardwareIfaceWidget::pwmChannel() const
{
  return pwm_channel_->getValue();
}

int VPitchHardwareIfaceWidget::pwmPeriodMinPitch() const
{
  return pwm_period_min_pitch_->getValue();
}

int VPitchHardwareIfaceWidget::pwmPeriodMaxPitch() const
{
  return pwm_period_max_pitch_->getValue();
}
}  // namespace ice
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
