#pragma once

#include "./base.hpp"
#include "../electrodynamics/electrodynamics.hpp"

namespace gui
{
namespace setup_assistant
{
namespace propulsion_system
{
class SpeedLimitWidget_Voltage : public SpeedLimitWidget_Base
{
  Q_OBJECT

public:
  explicit SpeedLimitWidget_Voltage(ElectrodynamicsWidget* electrodynamics);

  const char* name() const override;

  void onInit() override;

  bool isValid() override;

  double maxRotSpeed() const override;

private:
  ElectrodynamicsWidget* electrodynamics_;
};
}  // namespace propulsion_system
}  // namespace setup_assistant
}  // namespace gui
