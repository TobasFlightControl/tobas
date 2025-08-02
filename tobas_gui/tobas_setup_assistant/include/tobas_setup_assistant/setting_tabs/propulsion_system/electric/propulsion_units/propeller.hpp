#pragma once

#include "./base.hpp"
#include "tobas_setup_assistant/param_getters/double_spin_box.hpp"
#include "tobas_setup_assistant/param_getters/spin_box.hpp"

namespace gui
{
namespace sa
{
namespace propulsion
{
namespace electric
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

  /* Pitch length at the propeller tip [m] */
  double pitchLength() const;

  /* Pitch angle at the propeller tip [rad] */
  double pitchAngle() const;

  /* Minimum blade chord length [m] */
  double minChord() const;

  /* Maximum blade chord length [m] */
  double maxChord() const;

  /* Average blade chord length [m] */
  double meanChord() const;

private:
  ParamGetterWidget_SpinBox* num_blades_;
  ParamGetterWidget_SpinBox* diameter_;
  ParamGetterWidget_DoubleSpinBox* pitch_;
  ParamGetterWidget_SpinBox* min_chord_;
  ParamGetterWidget_SpinBox* max_chord_;
};
}  // namespace electric
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
