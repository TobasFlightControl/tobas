#pragma once

#include <QCheckBox>

#include <tobas_qt_tools/widgets/stacked_widget.hpp>

#include "../base_setting.hpp"
#include "./base.hpp"
#include "tobas_setup_assistant/frame_type.hpp"

namespace tobas
{
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

  void setFrameType(const FrameType& type);

  bool useBuiltinContrller() const;

  QString controllerPackage() const;
  QString pluginName() const;

  tobas::RcCommand acrobatModeCommand() const;
  tobas::RcCommand stabilizeModeCommand() const;
  tobas::RcCommand loiterModeCommand() const;

  YAML::Node staticParams() const;

private:
  FrameType frame_type_ = FrameType::kUndefined;

  QCheckBox* dont_use_builtin_ctrl_;
  tobas::qt::StackedWidget* stack_;

  BaseControllerWidget* widget(int index);
  const BaseControllerWidget* widget(int index) const;

  BaseControllerWidget* selected();
  const BaseControllerWidget* selected() const;

  void showCtrlWidgetWithFrameType(const FrameType& type);

private Q_SLOTS:
  void onDontUseBuiltinCtrlCheckBoxToggled(bool checked);
};
}  // namespace ctrl
};  // namespace sa
}  // namespace gui
}  // namespace tobas
