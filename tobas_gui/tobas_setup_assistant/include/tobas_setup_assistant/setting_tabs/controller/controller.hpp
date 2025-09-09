#pragma once

#include <tobas_qt_tools/widgets/stacked_widget.hpp>

#include "../base_setting.hpp"
#include "./base.hpp"
#include "tobas_setup_assistant/frame_type.hpp"

namespace gui
{
namespace sa
{
namespace ctrl
{
class ControllerWidget : public BaseSettingWidget
{
  Q_OBJECT

  using self = ControllerWidget;
  using super = BaseSettingWidget;

public:
  explicit ControllerWidget();

  const char* name() const override;
  const char* title() const override;
  const char* description() const override;

  void updateInternalDataStructures() override;
  bool isValid() override;

  YAML::Node dump() const override;
  void load(const YAML::Node& node) override;

  FrameType getFrameType() const;
  void setFrameType(const FrameType& type);

  QString controllerPackage() const;
  QString pluginName() const;

  tobas::RcCommand acrobatModeCommand() const;
  tobas::RcCommand stabilizeModeCommand() const;
  tobas::RcCommand loiterModeCommand() const;

  YAML::Node staticParams() const;

private Q_SLOTS:
  void setCurrentController(int index);

private:
  qt::StackedWidget* stack_;

  BaseControllerWidget* widget(int index);
  const BaseControllerWidget* widget(int index) const;

  BaseControllerWidget* selected();
  const BaseControllerWidget* selected() const;
};
}  // namespace ctrl
};  // namespace sa
}  // namespace gui
