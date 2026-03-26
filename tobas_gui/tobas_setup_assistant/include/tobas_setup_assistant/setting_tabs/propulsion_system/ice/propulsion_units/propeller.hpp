#pragma once

#include "./base.hpp"
#include "tobas_setup_assistant/param_getters/double_spin_box.hpp"
#include "tobas_setup_assistant/param_getters/int_range.hpp"
#include "tobas_setup_assistant/param_getters/spin_box.hpp"

namespace tobas
{
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

  /* Number of blades per propeller */
  int numBlades() const;

  /* Diameter of the propeller's rotational plane [m] */
  double diameter() const;

  /* Radius of the propeller's rotational plane [m] */
  double radius() const;

  /* Pitch length at the neutoral position [m] */
  double pitchLengthNeutoral() const;

  /* Pitch angle at the neutoral position [rad] */
  double pitchAngleNeutoral() const;

  /* Variable pitch angle limit around the neutoral position [rad] */
  st::Range<double> pitchAngleLimit() const;

  /* Maximum pitch angle rate [rad/s] */
  double maxPitchAngleRate() const;

  /* Minimum blade chord length [m] */
  double minChord() const;

  /* Maximum blade chord length [m] */
  double maxChord() const;

  /* Average blade chord length [m] */
  double meanChord() const;

private:
  ParamGetterWidget_SpinBox* num_blades_;
  ParamGetterWidget_SpinBox* diameter_;
  ParamGetterWidget_DoubleSpinBox* pitch_length_neutoral_;
  ParamGetterWidget_IntRange* pitch_angle_limit_;
  ParamGetterWidget_SpinBox* max_pitch_angle_rate_;
  ParamGetterWidget_SpinBox* min_chord_;
  ParamGetterWidget_SpinBox* max_chord_;
};
}  // namespace ice
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
}  // namespace tobas
