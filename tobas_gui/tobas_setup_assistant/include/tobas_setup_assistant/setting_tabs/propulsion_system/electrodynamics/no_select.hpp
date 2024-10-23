#pragma once

#include "./base.hpp"

namespace gui
{
namespace setup_assistant
{
namespace propulsion_system
{
class ElectrodynamicsWidget_NoSelect : public ElectrodynamicsWidget_Base
{
  Q_OBJECT

public:
  const char* name() const override;
  const char* description() const override;

  bool isValid() override;
  void copyFrom(const ElectrodynamicsWidget_Base* src) override;

  YAML::Node dump() const override;
  void load(const YAML::Node& node) override;

  std::pair<double, double> rotSpeedCoefs() const override;
};
}  // namespace propulsion_system
}  // namespace setup_assistant
}  // namespace gui
