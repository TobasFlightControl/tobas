#pragma once

#include "./base.hpp"
#include "../base.hpp"
#include "../electrodynamics/electrodynamics.hpp"
#include "../aerodynamics/aerodynamics.hpp"

namespace gui
{
namespace setup_assistant
{
class SpeedLimitWidget : public BaseSelectedLinkSettingWidget<SpeedLimitWidget>
{
  Q_OBJECT

  static constexpr char kMethodNameKey[] = "method_name";

public:
  explicit SpeedLimitWidget(ElectrodynamicsWidget* electrodynamics, AerodynamicsWidget* aerodynamics);

  const char* name() override;
  bool isValid() override;
  void copyFrom(const SpeedLimitWidget* src) override;

  YAML::Node dump() override;
  void load(const YAML::Node& node) override;

  /* Maximum rotation speed [rad/s] */
  double maxRotSpeed() const;

private:
  std::vector<SpeedLimitWidget_Base*> methods_;

  const SpeedLimitWidget_Base* selected() const;
};
}  // namespace setup_assistant
}  // namespace gui
