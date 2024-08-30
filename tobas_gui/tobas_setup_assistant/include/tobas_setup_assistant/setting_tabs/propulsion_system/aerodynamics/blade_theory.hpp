#pragma once

#include "./base.hpp"
#include "../propeller.hpp"
#include "../blade_theory.hpp"

namespace gui
{
namespace setup_assistant
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

  void onInit() override;

  bool isValid() override;
  void copyFrom(const AerodynamicsWidget_Base* src) override;

  YAML::Node dump() override;
  void load(const YAML::Node& node) override;

  double motorConst() const override;
  double momentConst() const override;
  double rotorDragCoef() const override;

private:
  PropellerWidget* propeller_;

  BladeTheory bladeTheory() const;
};
}  // namespace setup_assistant
}  // namespace gui
