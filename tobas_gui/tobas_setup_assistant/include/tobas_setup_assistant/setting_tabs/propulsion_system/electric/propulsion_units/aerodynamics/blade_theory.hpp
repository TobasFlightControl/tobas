#pragma once

#include "../blade_theory.hpp"
#include "../propeller.hpp"
#include "./base.hpp"

namespace gui
{
namespace sa
{
namespace propulsion
{
namespace electric
{
/**
 * @brief Unsteady Aerodynamic Parameter Estimation for Multirotor Helicopters [Nguyen+, 2019]
 */
class AerodynamicsWidget_BladeTheory : public AerodynamicsWidget_Base
{
  Q_OBJECT

public:
  explicit AerodynamicsWidget_BladeTheory(const PropellerWidget* propeller);

  const char* name() const override;
  const char* description() const override;

  bool isValid() override;
  void copyFrom(const AerodynamicsWidget_Base* src) override;

  YAML::Node dump() const override;
  void load(const YAML::Node& node) override;

  double motorConst() const override;
  double momentConst() const override;
  double dragConst() const override;

private:
  const PropellerWidget* const propeller_;

  BladeTheory bladeTheory() const;
};
}  // namespace electric
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
