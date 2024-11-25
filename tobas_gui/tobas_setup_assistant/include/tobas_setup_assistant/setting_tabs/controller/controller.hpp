#pragma once

#include <tobas_qt_tools/widgets/combo_box.hpp>
#include <tobas_qt_tools/widgets/stacked_widget.hpp>
#include <tobas_qt_tools/widgets/description_widget.hpp>

#include "tobas_setup_assistant/robot_info.hpp"
#include "../base_setting.hpp"
#include "../propulsion_system/propulsion_system.hpp"
#include "../fixed_wing/fixed_wing.hpp"
#include "./base.hpp"

namespace gui
{
namespace setup_assistant
{
class ControllerWidget : public BaseSettingWidget
{
  Q_OBJECT

  using self = ControllerWidget;
  using super = BaseSettingWidget;

  static constexpr char kTypeKey[] = "controller_type";

public:
  explicit ControllerWidget(
    RobotInfo& robot,
    const propulsion_system::PropulsionSystemWidget* propulsion_system,
    const fixed_wing::FixedWingWidget* fixed_wing);

  const char* name() const override;
  const char* title() const override;
  const char* description() const override;

  void onOpened() override;
  void updateInternalDataStructures() override;
  bool isValid() override;

  YAML::Node dump() override;
  void load(const YAML::Node& node) override;

  QString controllerPackage() const;
  QString pluginName() const;
  tobas::rc_command_t stabilizeModeCommand() const;
  tobas::rc_command_t acrobatModeCommand() const;
  YAML::Node staticParams() const;

  bool isCommandCompatible(tobas::rc_command_t command) const;

private Q_SLOTS:
  void setCurrentController(int index);

private:
  RobotInfo& robot_;
  const propulsion_system::PropulsionSystemWidget* propulsion_system_;
  const fixed_wing::FixedWingWidget* fixed_wing_;

  qt::ComboBox* type_;
  qt::StackedWidget* controllers_;
  qt::DescriptionWidget* description_;

  BaseControllerWidget* selected();
  const BaseControllerWidget* selected() const;
};
};  // namespace setup_assistant
}  // namespace gui
