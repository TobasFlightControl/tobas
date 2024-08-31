#pragma once

#include "tobas_setup_assistant/param_getters/spin_box.hpp"
#include "tobas_setup_assistant/param_getters/combo_box.hpp"

#include "./base.hpp"

namespace gui
{
namespace setup_assistant
{
class MotorWidget : public BaseSelectedLinkSettingWidget<MotorWidget>
{
  Q_OBJECT

public:
  explicit MotorWidget();

  const char* name() override;
  bool isValid() override;
  void copyFrom(const MotorWidget* src) override;

  YAML::Node dump() override;
  void load(const YAML::Node& node) override;

  /* Motor rotation direction (CW or CCW) */
  std::string direction() const;

  /* Kv [rad/s/V] */
  double kv() const;

  /* Internal resistance [Ω] */
  double internalResistance() const;

  /* Number of poles [-] */
  int numPoles() const;

  /* Time constant of the motor's response when increasing its rotational speed [s] */
  double timeConstUp() const;

  /* Time constant of the motor's response when decreasing its rotational speed [s] */
  double timeConstDown() const;

private:
  ParamGetterWidget_ComboBox* direction_;
  ParamGetterWidget_SpinBox* kv_;
  ParamGetterWidget_SpinBox* resistance_;
  ParamGetterWidget_SpinBox* num_poles_;
  ParamGetterWidget_SpinBox* time_const_up_;
  ParamGetterWidget_SpinBox* time_const_down_;
};
}  // namespace setup_assistant
}  // namespace gui
