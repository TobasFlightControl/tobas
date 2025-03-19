#pragma once

#include "./base.hpp"

namespace gui
{
namespace sa
{
namespace propulsion
{
namespace electric
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
}  // namespace electric
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
