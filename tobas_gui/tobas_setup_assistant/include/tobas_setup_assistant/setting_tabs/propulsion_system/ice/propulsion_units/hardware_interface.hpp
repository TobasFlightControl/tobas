#pragma once

#include "tobas_setup_assistant/param_getters/spin_box.hpp"
#include "./base.hpp"

namespace gui
{
namespace sa
{
namespace propulsion
{
namespace ice
{
class VPitchHardwareIfaceWidget : public BaseSelectedLinkSettingWidget
{
  Q_OBJECT

  using self = VPitchHardwareIfaceWidget;
  using super = BaseSelectedLinkSettingWidget;

public:
  explicit VPitchHardwareIfaceWidget();

  const char* name() const override;
  bool isValid() override;
  void copyFrom(const BaseSelectedLinkSettingWidget* src) override;

  YAML::Node dump() const override;
  void load(const YAML::Node& node) override;

  int pwmChannel() const;

  /* [us] */
  int pwmPeriodMinPitch() const;

  /* [us] */
  int pwmPeriodMaxPitch() const;

private:
  ParamGetterWidget_SpinBox* pwm_channel_;
  ParamGetterWidget_SpinBox* pwm_period_min_pitch_;
  ParamGetterWidget_SpinBox* pwm_period_max_pitch_;
};
}  // namespace ice
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
