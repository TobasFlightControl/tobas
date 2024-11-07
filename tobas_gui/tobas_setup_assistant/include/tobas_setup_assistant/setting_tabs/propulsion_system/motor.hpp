#pragma once

#include <tobas_drone_core/turning_direction.hpp>

#include "tobas_setup_assistant/param_getters/spin_box.hpp"
#include "tobas_setup_assistant/param_getters/combo_box.hpp"
#include "./base.hpp"

namespace gui
{
namespace setup_assistant
{
namespace propulsion_system
{
class MotorWidget : public BaseSelectedLinkSettingWidget
{
  Q_OBJECT

  static constexpr char kCWName[] = "CW";
  static constexpr char kCCWName[] = "CCW";

public:
  explicit MotorWidget();

  const char* name() const override;
  bool isValid() override;
  void copyFrom(const BaseSelectedLinkSettingWidget* src) override;

  YAML::Node dump() const override;
  void load(const YAML::Node& node) override;

  /* Motor rotation direction (CW or CCW) */
  tobas::turning_direction_t direction() const;

  /* Kv [rad/s/V] */
  double kv() const;

  /* Internal resistance [Ω] */
  double internalResistance() const;

  /* Number of poles [-] */
  int numPoles() const;

private:
  ParamGetterWidget_ComboBox* direction_;
  ParamGetterWidget_SpinBox* kv_;
  ParamGetterWidget_SpinBox* resistance_;
  ParamGetterWidget_SpinBox* num_poles_;
};
}  // namespace propulsion_system
}  // namespace setup_assistant
}  // namespace gui
