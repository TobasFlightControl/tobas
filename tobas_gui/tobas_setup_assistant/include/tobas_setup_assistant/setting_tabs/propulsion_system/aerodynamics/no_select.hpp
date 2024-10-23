#pragma once

#include "./base.hpp"

namespace gui
{
namespace setup_assistant
{
namespace propulsion_system
{
class AerodynamicsWidget_NoSelect : public AerodynamicsWidget_Base
{
  Q_OBJECT

public:
  const char* name() const override;
  const char* description() const override;

  bool isValid() override;
  void copyFrom(const AerodynamicsWidget_Base* src) override;

  YAML::Node dump() const override;
  void load(const YAML::Node& node) override;

  double motorConst() const override;
  double momentConst() const override;
  double rotorDragCoef() const override;
};
}  // namespace propulsion_system
}  // namespace setup_assistant
}  // namespace gui
