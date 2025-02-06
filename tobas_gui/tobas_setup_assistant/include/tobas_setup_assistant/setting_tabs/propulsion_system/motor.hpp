#pragma once

#include "tobas_setup_assistant/param_getters/spin_box.hpp"
#include "./base.hpp"

namespace gui
{
namespace setup_assistant
{
namespace propulsion
{
class MotorWidget : public BaseSelectedLinkSettingWidget
{
  Q_OBJECT

public:
  explicit MotorWidget();

  const char* name() const override;
  bool isValid() override;
  void copyFrom(const BaseSelectedLinkSettingWidget* src) override;

  YAML::Node dump() const override;
  void load(const YAML::Node& node) override;

  /* Kv [rad/s/V] */
  double kv() const;

  /* Internal resistance [Ω] */
  double internalResistance() const;

  /* Number of poles [-] */
  int numPoles() const;

private:
  ParamGetterWidget_SpinBox* kv_;
  ParamGetterWidget_SpinBox* resistance_;
  ParamGetterWidget_SpinBox* num_poles_;
};
}  // namespace propulsion
}  // namespace setup_assistant
}  // namespace gui
