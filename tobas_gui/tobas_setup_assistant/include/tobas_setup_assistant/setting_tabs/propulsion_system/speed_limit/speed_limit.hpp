#pragma once

#include "./base.hpp"
#include "../base.hpp"
#include "../motor.hpp"
#include "../aerodynamics/aerodynamics.hpp"

namespace gui
{
namespace setup_assistant
{
namespace propulsion
{
class SpeedLimitWidget : public BaseSelectedLinkSettingWidget
{
  Q_OBJECT

  static constexpr char kMethodNameKey[] = "method_name";

public:
  explicit SpeedLimitWidget(MotorWidget* motor, AerodynamicsWidget* aerodynamics);

  const char* name() const override;
  bool isValid() override;
  void copyFrom(const BaseSelectedLinkSettingWidget* src) override;

  YAML::Node dump() const override;
  void load(const YAML::Node& node) override;

  /* Maximum rotation speed [rad/s] */
  double maxRotSpeed() const;

private:
  std::vector<SpeedLimitWidget_Base*> methods_;

  const SpeedLimitWidget_Base* selected() const;
};
}  // namespace propulsion
}  // namespace setup_assistant
}  // namespace gui
