#pragma once

#include "./base.hpp"
#include "../../param_getters/spin_box.hpp"

namespace gui
{
namespace setup_assistant
{
namespace propulsion_system
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
  int numBlade() const;

  /* Diameter of the propeller's rotational plane [m] */
  double diameter() const;

  /* Radius of the propeller's rotational plane [m] */
  double radius() const;

  /* Chord length at 75% of the distance from the blade's center [m] */
  double bladeChord() const;

  /* Twist angle at 75% of the distance from the blade's center [rad] */
  double pitchAngle() const;

private:
  ParamGetterWidget_SpinBox* num_blade_;
  ParamGetterWidget_SpinBox* diameter_;
  ParamGetterWidget_SpinBox* blade_chord_;
  ParamGetterWidget_SpinBox* pitch_angle_;
};
}  // namespace propulsion_system
}  // namespace setup_assistant
}  // namespace gui
