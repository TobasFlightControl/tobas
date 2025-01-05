#pragma once

#include "tobas_setup_assistant/robot_info.hpp"
#include "../propulsion_system/propulsion_system.hpp"
#include "../fixed_wing/fixed_wing.hpp"
#include "./base.hpp"

namespace gui
{
namespace setup_assistant
{
class NonPlanarPIDWidget : public BaseControllerWidget
{
  Q_OBJECT

  static constexpr int kMinNumProp = 3;

public:
  explicit NonPlanarPIDWidget(
    RobotInfo& robot,
    const propulsion::PropulsionSystemWidget* propulsion_system,
    const fixed_wing::FixedWingWidget* fixed_wing);

  const char* name() const override;
  const char* description() const override;
  QString controllerPackage() const override;
  QString pluginName() const override;

  tobas::rc_command_t stabilizeModeCommand() const override;
  tobas::rc_command_t acrobatModeCommand() const override;

  YAML::Node staticParams() const override;

  YAML::Node dump() const override;
  void load(const YAML::Node& node) override;

  bool isApplicable() override;
  bool isValid() override;

private:
  RobotInfo& robot_;
  const propulsion::PropulsionSystemWidget* propulsion_system_;
  const fixed_wing::FixedWingWidget* fixed_wing_;
};
}  // namespace setup_assistant
}  // namespace gui
