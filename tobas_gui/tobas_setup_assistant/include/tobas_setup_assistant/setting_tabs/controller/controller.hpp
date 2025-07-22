#pragma once

#include <tobas_qt_tools/widgets/combo_box.hpp>
#include <tobas_qt_tools/widgets/stacked_widget.hpp>

#include "../base_setting.hpp"
#include "./base.hpp"
#include "tobas_setup_assistant/robot_info.hpp"

namespace gui
{
namespace sa
{
class ControllerWidget : public BaseSettingWidget
{
  Q_OBJECT

  using self = ControllerWidget;
  using super = BaseSettingWidget;

  static constexpr char kTypeKey[] = "controller_type";

public:
  explicit ControllerWidget(RobotInfo& robot);

  const char* name() const override;
  const char* title() const override;
  const char* description() const override;

  void updateInternalDataStructures() override;
  bool isValid() override;

  YAML::Node dump() const override;
  void load(const YAML::Node& node) override;

  QString controllerPackage() const;
  QString pluginName() const;

  tobas::RcCommand acrobatModeCommand() const;
  tobas::RcCommand stabilizeModeCommand() const;
  tobas::RcCommand loiterModeCommand() const;

  YAML::Node staticParams() const;

  bool isCommandCompatible(tobas::RcCommand command) const;

private Q_SLOTS:
  void setCurrentController(int index);

private:
  qt::ComboBox* type_;
  qt::StackedWidget* controllers_;

  BaseControllerWidget* widget(int index);
  const BaseControllerWidget* widget(int index) const;

  BaseControllerWidget* selected();
  const BaseControllerWidget* selected() const;
};
};  // namespace sa
}  // namespace gui
