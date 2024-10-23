#pragma once

#include "./base.hpp"
#include "../base.hpp"
#include "../aerodynamics/aerodynamics.hpp"
#include "../electrodynamics/electrodynamics.hpp"

namespace gui
{
namespace setup_assistant
{
namespace propulsion_system
{
class SpeedLimitWidget : public BaseSelectedLinkSettingWidget
{
  Q_OBJECT

  static constexpr char kMethodNameKey[] = "method_name";

public:
  explicit SpeedLimitWidget(AerodynamicsWidget* aerodynamics, ElectrodynamicsWidget* electrodynamics);

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
}  // namespace propulsion_system
}  // namespace setup_assistant
}  // namespace gui
