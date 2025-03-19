#pragma once

#include "./base.hpp"
#include "../propeller.hpp"
#include "../blade_theory.hpp"

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
  explicit AerodynamicsWidget_BladeTheory(PropellerWidget* propeller);

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
  PropellerWidget* propeller_;

  BladeTheory bladeTheory() const;
};
}  // namespace electric
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
