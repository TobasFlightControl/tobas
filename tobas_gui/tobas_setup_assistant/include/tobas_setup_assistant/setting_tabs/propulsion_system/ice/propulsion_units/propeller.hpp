#pragma once

#include "./base.hpp"
#include "tobas_setup_assistant/param_getters/int_range.hpp"
#include "tobas_setup_assistant/param_getters/spin_box.hpp"

namespace gui
{
namespace sa
{
namespace propulsion
{
namespace ice
{
class PropellerWidget : public BaseSelectedLinkSettingWidget
{
  Q_OBJECT

public:
  explicit PropellerWidget();

  const char* name() const override;
  bool isValid() override;
  void copyFrom(const BaseSelectedLinkSettingWidget* src) override;

  YAML::Node dump() const override;
  void load(const YAML::Node& node) override;

  /* [-] */
  int numBlade() const;

  /* [rad] */
  double pitchAngleRef() const;

  /* [rad] */
  tobas_std::Range<double> pitchAngleLimit() const;

  /* [rad/s] */
  double maxPitchAngleRate() const;

private:
  ParamGetterWidget_SpinBox* num_blade_;
  ParamGetterWidget_SpinBox* pitch_ref_;
  ParamGetterWidget_IntRange* pitch_limit_;
  ParamGetterWidget_SpinBox* max_pitch_rate_;
};
}  // namespace ice
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
